#!/usr/bin/python
# start using: py pressure_rec.py 169.254.65.43

import sys
import time
import datetime      # to access current date and time
import struct
from csv import writer  #to write csv files
import redpitaya_scpi as scpi


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
def compensate_humidity(h,t_fine):
    humidity = t_fine - 76800.0
    humidity = (h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
    humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
    if humidity > 100.0:
        humidity = 100.0
    elif humidity < 0.0:
        humidity = 0.0
    return humidity



rp_s = scpi.scpi('169.254.65.43')

period = 1 # seconds

if (len(sys.argv) > 2):
    led = int(sys.argv[2])
else:
    led = 0

# set GPIO pin direction:
#rp_s.tx_txt('DIG:PIN:DIR OUT,DIO0_N')

#init I2C
rp_s.tx_txt('I2C:DEV118 "/dev/i2c-0"')
print("Init I2C")
time.sleep(period/10.0)

#enable temperature and pressure measurement
# might not actually be required
# OSRS_h[2:0] in register 0xF2 to 001
# OSRS_T[2:0] in register 0xF4 to 001
# OSRS_P[2:0] in register 0xF4 to 001
# mode  [1:0] in register 0xF4 to 00
# 001 001 00 = 36(dec)
rp_s.tx_txt('I2C:S:W242 5')
rp_s.tx_txt('I2C:S:W244 180')
time.sleep(period/10.0)

# read compensation registers
# temperature
print('temperature coefficients')
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
print('pressure coefficients')
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
print('humidity coefficients')
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



while 1:
    # LED on - indicating measurement
    time.sleep(period)
    rp_s.tx_txt('DIG:PIN LED' + str(led) + ',' + str(1))

    # force measurement
    measurement_time = time.time()
    rp_s.tx_txt('I2C:S:W244 181') #37
    time.sleep(period/10.0)

    #read out all measurements at once & format into pressure, temperature and humidity
    rp_s.tx_txt('I2C:S:R247:B8')
    m = rp_s.rx_txt().strip('{').strip('}')
    m = m.split(',')
    byte_array = []
    for s in m:
        byte_array.append(int(s))
    p = byte_array[0]*2**16 + byte_array[1]*2**8 + byte_array[2]


    t = byte_array[3]*2**16 + byte_array[4]*2**8 + byte_array[5]


    h = byte_array[6]*2**8 + byte_array[7]


    # measurement compensation and formatting of output
    t_fine = compensate_temperature(t)
    temperature_output = str(round(float(t_fine)/5120.0,2))
    pressure = compensate_pressure(p,t_fine)
    pressure_output = str(round(pressure,2) )
    humidity = compensate_humidity(h,t_fine)
    humidity_output = str(round(humidity,2) )

    # print data
    print(str(datetime.datetime.fromtimestamp(measurement_time)) + ' - Temperature: ' + temperature_output + ' degC - Pressure: ' + pressure_output + ' Pa - Humidity: ' + humidity_output + ' %')
    # save data in file - change file name every day to avoid massive files
    list_data = [str(measurement_time),temperature_output,pressure_output,humidity_output]
    file =  open('Pressure_and_temperature_rec_' + time.strftime("%Y_%m_%d", time.localtime()) +'.csv', 'a', newline='')    # a - append
    writer_object = writer(file)
    writer_object.writerow(list_data) 
    file.close()

    # turn LED of after 1 second
    time.sleep(period)
    rp_s.tx_txt('DIG:PIN LED' + str(led) + ',' + str(0))

    # # one measurement every minute
    time.sleep(period*58)
