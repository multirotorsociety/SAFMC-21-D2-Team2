import serial
import time

ser = serial.Serial('/dev/ttyACM0')
ser.write(b'1')
time.sleep(2)
ser.write(b'0')
ser.close()
