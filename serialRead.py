import serial as ser
import time

ser = serial.Serial('/dev/ttyACM0',115200, timeout=10) # IMPORTANT, CHANGE ACM0 to USB0 OR OTHER AS APPROPIRATE
ser.open()
data = ser.read()
while True:
    time.sleep(0.5)
    print(data)
    time.sleep(0.5)
