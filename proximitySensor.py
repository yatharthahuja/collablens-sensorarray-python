import threading
import serial
import numpy as np
from time import *
from time import sleep
from events import Events
import statistics


#
##########################
# Proximity Sensor: Lidar
##########################
#
class proximitySensor:

    # Class Variables
    port = "/dev/ttyUSB0"
    baudRate = 115200
    ser = serial.Serial(port, baudRate,timeout=0) # mini UART serial device

    # The init method or constructor
    def __init__(self, port, baudRate):
        # Instance Variable
        self.port = port
        self.baudRate = baudRate

    def flushSensor(self):
        ser = serial.Serial(self.port, self.baudRate,timeout=0)
        if ser.isOpen() == False:
            ser.open() # open serial port if not open
        ser.flushInput()
        ser.flushOutput()
        sleep(0.2)
        ser.close()

    # To give distance detected
    def getDistance(self):
        distance = 0.0
        ser = serial.Serial(self.port, self.baudRate,timeout=0) # mini UART serial device
        if ser.isOpen() == False:
            ser.open() # open serial port if not open
        while(1):
            counter = ser.in_waiting # count the number of bytes of the serial port
            if counter > 8:
                bytes_serial = ser.read(9) # read 9 bytes
                ser.reset_input_buffer() # reset buffer
                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                    distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                    distance /= 100.0
                    return distance
        ser.close() # close the serial port
        #print('Distance: {0:2.2f} m'. format(distance)) # print sample data
        return 100.0