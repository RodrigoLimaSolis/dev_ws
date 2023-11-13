import serial.tools.list_ports
import threading
import time

print(list(serial.tools.list_ports.comports()))