import serial as ser
import time

ser = serial.Serial('/dev/ttyACM0',115200, timeout=10) # IMPORTANT, CHANGE ACM0 to USB0 OR OTHER AS APPROPIRATE
ser.open()
data = ser.read()
while True:
    new_file = open('info_file.txt', 'w')
    new_file.write(data)
