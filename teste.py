import serial
import threading
import time

usb = serial.Serial('/dev/ttyACM0', 57600, timeout=100, dsrdtr=False)

#teste

usb.write(b"LT E1 RD00 GR00 BL100")
usb.write(b"MT0 ME1")
usb.write(b"MT0 MC MD0 AT2000 DT2000 V10")

#Leitura comandos Sobot
def Read_Serial():
    global RobotIndex, mode
    while True:
        command = usb.readline() # Read data
        if(command != b''):
            print("Commando recebido",command) 

leitura = threading.Thread(target=Read_Serial, daemon=True)
leitura.start()


while True:
    usb.write(b"MT0 D45 DF L RI100 V10")
    time.sleep(0.1)
    usb.write(b"MT0 MS")

    time.sleep(4)
    usb.write(b"MT0 MS")
    time.sleep(3)

    break
