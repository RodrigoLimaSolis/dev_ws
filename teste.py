import serial
import time 
#Comunicação com o SOBOT
usb = serial.Serial('/dev/ttyACM0', 9600, timeout=0, dsrdtr=False)

usb.write(b"MT0 E1")
for a in range(10):
    usb.write(b"MT0 D1 DF L RI382 V7")
