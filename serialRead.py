import serial as ser
import time
import bitdotio
import pandas as pd

b = bitdotio.bitdotio('postgresql://x-1-x:v2_3sAPc_xm6m98XVFz62zt3gBQntMJD@db.bit.io/x-1-x/AirQuality')
ser = serial.Serial('/dev/ttyACM0',115200, timeout=10) # IMPORTANT, CHANGE ACM0 to USB0 OR OTHER AS APPROPIRATE
ser.open()
data = ser.read()


while True:
    new_file = open('info_file.txt', 'a')
    new_file.write(data)
# Unfinished, more functionality is being added to write this data into a database
