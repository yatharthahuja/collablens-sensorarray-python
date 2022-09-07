import threading
import serial
import numpy as np
from time import *
from time import sleep
from events import Events
import statistics

#
##########################
# Proximity Sensor Array: Multiple Lidar Sensors in an Array
##########################
#
class sensorArray:

    # Class Variables
    firstPort = "/dev/ttyUSB0"
    #ports = ["/dev/ttyUSB0"]
    baudRate = 115200
    sensors = []
    numSensors = 3
    objDetected = False
    detectedVelocities = []
    humanDistThreshold = 0.7#m minimum distance of human/dynamic interference from sensors for it to be detected
    objDistThreshold = 0.45#m minimum distance of bag from sensors for it to be detected
    timeThreshold = 0.5#s maximum time after which the proceeding sensor gives-up the wait for dropping object to be detected
    holdThreshold = 1.0#s minimum time for which bag is to be held at drop-height
    dropTimeThreshold = 5.0#s maximum time after which the proceeding sensor gives-up the wait for drop to inititate (drop abandon)
    sensorDisplacement = 0.8#m distance between sensors at installation
    obstructionCheck = False# whether to execute human/dynamic obstruction check
    objVelThreshold = 3.0#m/s velocity should be greater than 1.8 or something and place the sensor at a lower level
    events = Events()

    msg = ""# message for the display intermiddiate

    # The init method or constructor
    def __init__(self, firstPort, numSensors, baudRate):
        self.firstPort = firstPort
        self.numSensors = numSensors
        self.baudRate = baudRate
        i = 0
        while i < self.numSensors :
            self.sensors.append(proximitySensor("/dev/ttyUSB"+str(i), 115200))
            i += 1

    # For debugging by getting trigger distances
    def trigDistance(self):
        i = 0
        while i < self.numSensors:
            print("Distance at USB Port ["+str(i+1)+"] is: "+str(self.sensors[i].getDistance()))
            i += 1

    # To flush all ports to erase buffer data
    def flushSensorInputs(self):
        i = 0
        while i < self.numSensors:
            self.sensors[i].flushSensor()
            i += 1

    def modSubtraction(self, a, b):
        if a > b:
            return float(a)-float(b)
        return float(b)-float(a)

    # To check if any human is standing in front of the array
    def checkHuman(self):
        humanDetected = False
        sensors = self.sensors[1:] # some might not be 1.8m tall
        count = 1 # sensor number
        totalDistance = 0
        for sensor in sensors:
            distance = sensor.getDistance()
            totalDistance += distance
            avgDistance = float(totalDistance)/float(count)
            if (distance < self.humanDistThreshold) and (self.modSubtraction(avgDistance, distance)/avgDistance < 0.1):
                humanDetected = True
                break
            count += 1
        return humanDetected

    def registerNetworkSend(self,network_send_function):
        self.events.on_change+=network_send_function

    # Intermiddiate function to pass screen display messages to network function
    def sendPacket(self):
        # code to serialize.
        return self.msg

    # To get velocity betweeen each sensor intervals
    def getVelocity(self):

        if self.numSensors < 3:
            print("Hardware doesn't support this functionality: Please Upgrade!")
            return 0

        while(True):#or input == 'enter'

            """
            if input == enter:
                break
            """

            self.flushSensorInputs()
            #print("flushed!")

            """
            if self.checkHuman() and self.obstructionCheck:
                print("Human Detected!")
                self.msg = "Human Detected!"
                print("**********************")
                sleep(1)
                continue
            """

            if self.sensors[0].getDistance() < self.objDistThreshold:
                timeCurrent = time()
                print("Object Detected at sensor 1")
                self.msg = "Object Detected at sensor 1"
                message1 = threading.Thread(target = self.events.on_change, args = ("Object Detected at sensor 1",))
                message1.start()

                objHeld = True
                while time() - timeCurrent < self.holdThreshold:
                    if self.sensors[0].getDistance() > self.objDistThreshold:
                        objHeld = False
                        break

                if not objHeld:
                    print("Object not properly held at drop-height for the stipulated time")
                    self.msg = "Object not properly held at drop-height for the stipulated time"
                    message2 = threading.Thread(target = self.events.on_change, args = ("Object not properly held at drop-height for the stipulated time",))
                    message2.start()
                    print("**********************")
                    continue

                index = 2
                self.detectedVelocities = []
                print("Object ready to be dropped")
                self.msg = "Object ready to be dropped"
                message3 = threading.Thread(target = self.events.on_change, args = ("Object ready to be dropped",))
                message3.start()
                #self.events.on_change("Object ready to be dropped")

                objDropped = False

                while time() - timeCurrent < self.dropTimeThreshold:
                    if self.sensors[1].getDistance() < self.objDistThreshold:
                        objDropped = True
                        break

                if not objDropped:
                    print("Object not dropped at all in expected time!")
                    self.msg = "Object not dropped at all in expected time!"
                    message4 = threading.Thread(target = self.events.on_change, args = ("Object not dropped at all in expected time!",))
                    message4.start()
                    sleep(2)
                    continue

                while(True):
                    if self.sensors[1].getDistance() < self.objDistThreshold:
                        print("Object Detected at sensor 2")
                        self.msg = "Object Detected at sensor 2"
                        message5 = threading.Thread(target = self.events.on_change, args = ("Object detected at sensor 2",))
                        message5.start()
                        timeCurrent = time()
                        break

                # beyond 2 sensors:
                while index < self.numSensors:
                    self.objDetected = False
                    while time() - timeCurrent < self.timeThreshold:
                        if self.sensors[index].getDistance() < self.objDistThreshold:
                            timePrevious = timeCurrent
                            timeCurrent = time()
                            self.detectedVelocities.append(self.sensorDisplacement/
                                                      (timeCurrent-timePrevious))
                            if (self.sensorDisplacement/(timeCurrent-timePrevious)<self.objVelThreshold):
                                break
                            self.objDetected = True
                            print("Object Detected at sensor "+str(index + 1)+"    "+str(timeCurrent-timePrevious))
                            self.msg = "Object Detected at sensor "+str(index + 1)+"    "+str(timeCurrent-timePrevious)
                            message6 = threading.Thread(target = self.events.on_change, args = ("Object Detected at sensor "+str(index + 1)+"    "+str(timeCurrent-timePrevious),))
                            message6.start()
                            break

                    if self.objDetected == False:
                        print("Objected not detected at sensor "+str(index + 1)+" in expected time")
                        print("False drop!")
                        self.msg = "False drop!"
                        message7 = threading.Thread(target = self.events.on_change, args = ("False Drop!",))
                        message7.start()
                        print("**********************")
                        break
                    index += 1

            if self.objDetected:
                print("velarr: "+str(self.detectedVelocities))
                message8 = threading.Thread(target = self.events.on_change, args = ("Drop Completed",statistics.mean(self.detectedVelocities),))
                message8.start()
                # since time detected at 1st sensor does not equate to actual time of drop
                # hence velocity in that interval is discarded
                print("**********************")
                self.objDetected = False
                #return self.detectedVelocities

            sleep(2.0)

if __name__ == "__main__":
    array = sensorArray("/dev/ttyUSB0", 3, 115200)
    print("initiated")
    #while True:
    #    print(array.trigDistance())
    print(array.getVelocity())
