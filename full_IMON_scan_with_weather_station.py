
import pyvisa
import time
import datetime      # to access current date and time
import sys
import csv
import zmq
import threading
import queue
import numpy as np
import datetime
from pyvisa.errors import VisaIOError
import struct
import redpitaya_scpi as scpi

def parameters():
    startWav = 1594#starting wl
    stopWav =  1568.5 #stoping wl
    stepSize = -0.01 #step size in nm, make sure to change sign depending on direction of travel
    FWHM = 0.44 # TF FWHM in nm
    noWMSamples = 5 #number of recorded samples for the WM. WM sample rate is 2 samples/sec
    maxDifference = 1 #RMS difference to check for TEC spikes
    WMPowerLimits = [30.2e-6, 35.2e-6] #WM input power limits in Watts to check for FBG peak height
    PSVoltage = 1.0 #inital power source voltage, DO NOT SET TO MORE THAN 5V!!!! 
    redPitayaIP = '169.254.65.43' #Red Pitaya IP address as shown on SCPI server
    return startWav,stopWav,stepSize,FWHM,noWMSamples,maxDifference,WMPowerLimits,PSVoltage,redPitayaIP

def IMONrecording(): #IMON recording subroutine
    global socket,isRecording
    print("Recording IMON...")
    IMONout = [0]
    while isRecording:
        socket.send(b"true")

        #Get the reply.
        message = socket.recv()
        string = message.decode('utf-8')
        number = float(string)/1e5
        if number==IMONout[-1]: #continiously records SV stream and discards repeated values
            continue
        IMONout.append(number)
    print('IMON recorded')
    q1.put(IMONout)

def WMrecording(): #reference recording subroutine
    global WM,noWMSamples,isRecording
    print("Recording WM...")
    WMout = []
    for j in range(noWMSamples):
        wl = WM.query_ascii_values(":MEASure:SCALar:POWer:WAVelength? MAX, MIN")
        wl = round(wl[0]*1e9,5)
        WMout.append(wl)
    isRecording = False
    print('WM recorded')
    q2.put(WMout)

def compensate_temperature(t):
    # compoensation Temperature
    raw_temperature = float(t/16)  # lowest 4 bits get dropped
    var1 = (raw_temperature/16384.0 - dig_T1/1024.0) * dig_T2
    var2 = ((raw_temperature/131072.0 - dig_T1/8192.0) * (raw_temperature/131072.0 - dig_T1/8192.0)) * dig_T3
    t_fine = var1 + var2
    
    return t_fine

def compensate_pressure(p,t_fine):
    # compensation pressure
    adc = float(p/16)  # lowest 4 bits get dropped
    var1 = float(t_fine)/2.0 - 64000.0
    var2 = var1 * var1 * dig_P6/32768.0
    var2 = var2 + var1 * dig_P5*2.0
    var2 = var2/4.0 + dig_P4*65536.0
    var1 = (dig_P3 * var1 * var1/524288.0 + dig_P2 * var1)/524288.0
    var1 = (1.0 + var1/32768.0) * dig_P1
    if not var1:  # avoid exception caused by division by zero
        raise ArithmeticError(
            "Invalid result possibly related to error while reading the calibration registers"
        )
    pressure = 1048576.0 - adc
    pressure = (pressure - var2/4096.0) * 6250.0/var1
    var1 = dig_P9 * pressure * pressure/2147483648.0
    var2 = pressure * dig_P8/32768.0
    pressure = pressure + (var1 + var2 + dig_P7) / 16.0
    pressure /= 100
    
    return pressure
def compensate_humidity(h,t_fine):# compoensation Humidity
    humidity = t_fine - 76800.0
    humidity = (h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
    humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
    if humidity > 100.0:
        humidity = 100.0
    elif humidity < 0.0:
        humidity = 0.0
    return humidity

#load configurable parameters
startWav,stopWav,stepSize,FWHM,noWMSamples,maxDifference,WMPowerLimits,PSVoltage,redPitayaIP = parameters()
context = zmq.Context()
#  Connect to Socket to talk to LabVIEW server

print("\nConnecting to LabVIEW server…")
print("If nothing happens check Labview")
socket = context.socket(zmq.REQ,)
socket.connect("tcp://localhost:5555")
#Send nudge to server
socket.send(b"true")

#Get the reply.
message = socket.recv()
print('Connected to LabVIEW Server')

rm = pyvisa.ResourceManager()

# try connecting to wavemeter
try:
     print('\nConnecting to Agilent WM')
     WM= rm.open_resource("TCPIP0::86122C-0269::inst0::INSTR")
     WM.read_termination = '\n' #choose \r for serial connection or \n for TCP/IP 
     WM.write_termination = '\n' 
     WM.timeout = 2000
     WM.write("*CLS")
     WM.write("*RST")
     finishedOPeration = 0
     WM.write(":CORR:DEV BRO")
     while finishedOPeration==0: 
        finishedOPeration = int(WM.query("*OPC?"))
     WM.write(":CORR:MED VAC")
     finishedOPeration = 0
     WM.write(":CORR:ELEV 40")
     while finishedOPeration==0: 
        finishedOPeration = int(WM.query("*OPC?"))
     WM.write(":UNIT W")
     WM.write(":DISPlay:WINDow2:TRACe:SCALe:LEFT 1520E-9")
     WM.write(":DISPlay:WINDow2:TRACe:SCALe:RIGHT 1600E-9")

     print("Connected to " + WM.query("*IDN?"))
     time.sleep(1)

except VisaIOError:
     print("Error! Can't connect to Agilent 86122C")
     input("\nPress Enter to Exit...")
     sys.exit()

# try connecting to tunable filter
try:
    print('\nConnecting to Santec TF')
    #TF= rm.open_resource("TCPIP0::192.168.11.1::5000::SOCKET") #use this for TCP/IP connection
    TF= rm.open_resource("ASRL28::INSTR") #use this for serial connection
    TF.read_termination = '\r' #choose \r for serial connection or \n for TCP/IP 
    TF.write_termination = '\r' 
    TF.timeout = 2000
    TF.write("*CLS")
    TF.write("*RST")
    stopedMoving = 0
    TF.write(":BAND " + str(FWHM) +"nm")
    while stopedMoving==0: 
        stopedMoving = int(TF.query("*OPC?"))
    
    print("Connected to " + TF.query("*IDN?"))
    time.sleep(1)

except VisaIOError:
    print("Error! Can't connect to Santec OTF-980 TF, check port number")
    input("\nPress Enter to Exit...")
    sys.exit()

# try connecting to power source to control attenuator 
try:
    print('\nConnecting to TENMA power source')
    PS = rm.open_resource("ASRL7::INSTR")
    PS.timeout = 2000
    PS.read_termination = '\n'
    PS.write_termination = '\n'
    if PSVoltage > 5:
        print("Voltage setting is too high!! Change to less than 5V and more than 0 and start scan again")
        input("\nPress any key to Exit...")
        sys.exit()
    print("Connected to: " + PS.query("*IDN?"))
    PS.write("VSET1:" + str(PSVoltage))
    time.sleep(1)
    PS.write('ISET1:0.001')
    PS.write("OUT1")
    time.sleep(0.5)
    PS.write('OCP1')
    print('Voltage set to: ' + PS.query('VSET1?') + 'V')

except VisaIOError:
    print("Error! Can't connect to Tenma 72-2550 PS")
    input("\nPress any key to Exit...")
    sys.exit()

try:
    print('\nConnecting to Red Pitaya')
    rp_s = scpi.scpi(redPitayaIP)
    #init I2C for Red Pitaya
    rp_s.tx_txt('I2C:DEV118 "/dev/i2c-0"')
    time.sleep(0.1)

    
    rp_s.tx_txt('I2C:S:W242 5')
    rp_s.tx_txt('I2C:S:W244 180')
    print("Connected to Red Pitaya")
    time.sleep(0.1)

    # read compensation registers
    # temperature
    print('Temperature coefficients:')
    rp_s.tx_txt('I2C:S:R136:W') #0x88
    dig_T1 = float(struct.unpack('H',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_T1)
    rp_s.tx_txt('I2C:S:R138:W')  #0x8A
    dig_T2 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_T2)
    rp_s.tx_txt('I2C:S:R140:W')  #0x8C
    dig_T3 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_T3)
    # pressure
    print('Pressure coefficients:')
    rp_s.tx_txt('I2C:S:R142:W')  #0x8E
    dig_P1 = float(struct.unpack('H',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P1)
    rp_s.tx_txt('I2C:S:R144:W')  #0x90
    dig_P2 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P2)
    rp_s.tx_txt('I2C:S:R146:W')  #0x92
    dig_P3 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P3)
    rp_s.tx_txt('I2C:S:R148:W')  #0x94
    dig_P4 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P4)
    rp_s.tx_txt('I2C:S:R150:W')  #0x96
    dig_P5 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P5)
    rp_s.tx_txt('I2C:S:R152:W')  #0x98
    dig_P6 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P6)
    rp_s.tx_txt('I2C:S:R154:W')  #0x9A
    dig_P7 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P7)
    rp_s.tx_txt('I2C:S:R156:W')  #0x9C
    dig_P8 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P8)
    rp_s.tx_txt('I2C:S:R158:W')  #0x9E
    dig_P9 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_P9)
    # humidity coefficients
    print('Humidity coefficients:')
    rp_s.tx_txt('I2C:S:R161:W')  #0xA1
    rx_value = rp_s.rx_txt()
    low8bit = int(rx_value) & 0xFF
    dig_H1 = float(struct.unpack('B',low8bit.to_bytes(1,'little'))[0])
    print(dig_H1)
    rp_s.tx_txt('I2C:S:R225:W')  #0xE1
    dig_H2 = float(struct.unpack('h',int(rp_s.rx_txt()).to_bytes(2,'little'))[0])
    print(dig_H2)
    rp_s.tx_txt('I2C:S:R227:W')  #0xE3
    rx_value = rp_s.rx_txt()
    low8bit = int(rx_value) & 0xFF
    dig_H3 = float(struct.unpack('B',low8bit.to_bytes(1,'little'))[0])
    print(dig_H3)
    # calculate dig_H4 and dig_H5
    rp_s.tx_txt('I2C:S:R228:B3')  # read registers E4h, E5h, and E6h at once
    m = rp_s.rx_txt().strip('{').strip('}')
    m = m.split(',')
    byte_array = []
    for s in m:
        byte_array.append(int(s))
    e4 = byte_array[0]
    e5 = byte_array[1]
    e6 = byte_array[2]
    dig_H4 = (e4 * 16) + (e5 & 0x0F)
    dig_H5 = (e6 * 16) + (e5 >> 4)
    print(dig_H4)
    print(dig_H5)
    rp_s.tx_txt('I2C:S:R231:W')  #0xE7
    rx_value = rp_s.rx_txt()
    low8bit = int(rx_value) & 0xFF
    dig_H6 = float(struct.unpack('b',low8bit.to_bytes(1,'little'))[0])
    print(dig_H6)
    time.sleep(0.1)
except:
    print("Error! Can't connect to Red Pitaya, make sure SCPI server is running and IP address is matching")
    input("\nPress any key to Exit...")
    sys.exit()


noSteps = round((stopWav-startWav)/stepSize)
eta = ((noWMSamples/2)*noSteps)
eta = round(eta/60,2)
print("\nStarting IMON scan...")
print("Number of steps: " + str(noSteps))
print("ETA: " + str(eta) + "mins")
current_time = datetime.datetime.now()
timeString = str(current_time.day) +"_"+str(current_time.month)+"_"+str(current_time.year)+"_"+str(current_time.hour)+"_"+str(current_time.minute)+"_"+str(current_time.second)

headers = ['Pressure','Temperature','Humidity','WM Power','WM WL', 'IMON WL','WM RMS','IMON RMS'] #CSV file column headers
q1 = queue.Queue()
q2 = queue.Queue()
with open("RecordedOutput"+timeString+".csv","w+",newline="") as csvfile :   #Opens CSV file to record data
    WMfile = csv.writer(csvfile) 
    WMfile.writerow(headers)        #Write column headers to CSV file

    step = 0
    repeat = False
    repeatCounter = 0
    RMSdiffArray = []
    
    while step < noSteps+1 :
        p1 = threading.Thread(target=IMONrecording)
        p2 = threading.Thread(target=WMrecording)
        if repeat == False:
            print('\nStep: '+str(step) + '/' + str(noSteps))
            print("\nMoving to WL "+str(startWav)+"nm")
            stopedMoving = 0
            TF.write(":WAV "+str(startWav)+"nm")#moves TF to WL value and checks if move was performed before continuing
            while stopedMoving==0: 
                stopedMoving = int(TF.query("*OPC?")) 
            
        pow = WM.query_ascii_values(":MEASure:SCALar:POWer? MAX, MIN") #cheks power recieved by the WM
       
        print("Power: " + str(round(pow[0]*1e6,2)) +"uW")
        while pow[0] < WMPowerLimits[0]: #loops while the minimum WM power limit is exceeded
            print("WM Power low...")
            currentVoltage =  PS.query('VSET1?')
            time.sleep(0.5)
            newVoltage = round(float(currentVoltage) - 0.01,2) # PS voltage is decreased by 0.01V
            if newVoltage < 0: #checks if PS voltage is 0 and if so aborts scan 
                print('PW voltage min limit reached, aborting scan... ')
                input("\nPress any key to Exit...")
                sys.exit()
            
            PS.write("VSET1:" + str(newVoltage))
            time.sleep(1)
            pow = WM.query_ascii_values(":MEASure:SCALar:POWer? MAX, MIN") #checks power recieved by the WM
            print('PS Voltage set to:' + PS.query('VSET1?'))
            print("Power: " + str(round(pow[0]*1e6,2)) +"uW")
        while pow[0] > WMPowerLimits[1]: #loops while the maximum WM power limit is exceeded
            print("WM Power high...")
            currentVoltage =  PS.query('VSET1?')
            time.sleep(0.5)
            newVoltage = round(float(currentVoltage) + 0.01,2) # PS voltage is increased by 0.01V
            if newVoltage > 5: #checks if PS voltage is 5V and if so aborts scan 
                print('PW voltage max limit reached, aborting scan... ')
                input("\nPress any key to Exit...")
                sys.exit()
            
            PS.write("VSET1:" + str(newVoltage))
            time.sleep(1)
            pow = WM.query_ascii_values(":MEASure:SCALar:POWer? MAX, MIN") #checks power recieved by the WM
            print('PS set to: ' + PS.query('VSET1?') + 'V')
            print("Power: " + str(round(pow[0]*1e6,2)) +"uW")
        # force pressure sensor measurement
        rp_s.tx_txt('I2C:S:W244 181') #37
        time.sleep(0.1)

        #read out all measurements from the BME280 sensor at once & format into pressure, temperature and humidity
        rp_s.tx_txt('I2C:S:R247:B8')
        m = rp_s.rx_txt().strip('{').strip('}')
        m = m.split(',')
        byte_array = []
        for s in m:
            byte_array.append(int(s))
        p = byte_array[0]*2**16 + byte_array[1]*2**8 + byte_array[2]


        t = byte_array[3]*2**16 + byte_array[4]*2**8 + byte_array[5]


        h = byte_array[6]*2**8 + byte_array[7]


        # BME280 measurement compensation and formatting of output
        t_fine = compensate_temperature(t)
        temperature_output = str(round(float(t_fine)/5120.0,2))
        pressure = compensate_pressure(p,t_fine)
        pressure_output = str(round(pressure,2) )
        humidity = compensate_humidity(h,t_fine)
        humidity_output = str(round(humidity,2) )
        print( 'Temperature: ' + temperature_output + 'degC - Pressure: ' + pressure_output + 'hPa - Humidity: ' + humidity_output + '%')
        isRecording = True
        p1.start() #starts IMON acquisition
        p2.start() #starts WM acquisition
        p1.join()  #joins both processes to run on parallel
        p2.join()
        IMONout = q1.get() 
        WMout = q2.get()
        IMONmean = np.mean(IMONout[2:])
        IMONrms = np.sqrt(np.mean(np.square(IMONout[2:]-IMONmean)))
        WMmean = round(np.mean(WMout),5)
        WMrms = np.sqrt(np.mean(np.square(WMout-WMmean)))
        
        if step > 0:
            rmsDifference = IMONrms - prevRMS
        else:
            rmsDifference = 0
        print('Max allowed RMS difference = ' + str(maxDifference))
        print('Imon RMS difference= ' + str(rmsDifference))
        if rmsDifference < maxDifference: #checks if a TEC spike occured based on the RMS difference criteria
            WMfile.writerow([pressure_output,temperature_output,humidity_output,pow[0],WMmean,IMONmean,WMrms,IMONrms])    #saves data to csv file
            startWav = round(startWav + stepSize,3) #increases wavelength by step size and iterate
            step += 1
            repeat = False
            prevRMS = IMONrms
            repeatCounter = 0
            RMSdiffArray = []
        else: 
            if repeatCounter<5:
                print('TEC spike detected, waiting 20sec...') #if TEC spike detected waits for 20 seconds and tries again
                time.sleep(20)
                repeat = True
                RMSdiffArray.append(rmsDifference)
                # print(RMSdiffArray)
                repeatCounter += 1
            else :       # if more than 5 attempts are made then the RMS difference criteria is adjusted and process starts again
                maxDifference = round(np.mean(RMSdiffArray),6)
                
                print('\nmax RMS difference adjusted to: ' + str(maxDifference))
                repeatCounter = 1
                RMSdiffArray = []
            
            
            print('\nRepeating step: ' + str(step) + ' ' + str(repeatCounter) + '/5 times')
            
            

        

print("\nFinished Scan")
print("Number of steps: " + str(noSteps))
WM.close()
TF.close()