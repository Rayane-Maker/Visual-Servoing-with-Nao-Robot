import time
import sys
import os
import numpy as np
from threading import Thread
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from utils.tasks_utils import*

class Measure:
    def __init__(self):
        self.measure = None

    # To override with custom child class (custom sensor)
    def getMeasure(self):
        return self.measure

class Actuator:
    def __init__(self):
        return None

    # To override with custom child class (custom actuator)
    def control(self, command):
        return None


class PIDController(LoopTask):

    def __init__(self, setPoint, kp, ki, kd, integralLimit, frequency=20, trackbars_window=None, trackbarCoeffs = None):
        super(PIDController, self).__init__()
        self.input = np.zeros((len(setPoint),1))
        self.err = np.zeros((len(setPoint),1))
        self.oldErr = np.zeros((len(setPoint),1))
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = np.zeros((len(setPoint),1))
        self.derivative = np.zeros((len(setPoint),1))
        self.command = np.zeros((len(setPoint),1))
        self.setSetPoint(setPoint)
        self.trackbarCoeffs = trackbarCoeffs if trackbarCoeffs is not None else np.ones((3,1))
        self.trackbars_window = trackbars_window
        self.createTrackbars()
        self.setKp(self.kp)
        self.setKi(self.ki)
        self.setKd(self.kd)
        self.setIntegralLimit(integralLimit)
        self.setFrequency(frequency)
        
        
    
    

    ######### Param Setters ##############
    def setSetPoint(self, setPoint):
        self.setPoint = setPoint

        
    def setKp(self, kp):
        self.kp = kp
        self.updateTrackbars()

    def setKi(self, ki):
        self.ki = ki
        self.updateTrackbars()

    def setKd(self, kd):
        self.kd = kd
        self.updateTrackbars()


    def setIntegralLimit(self, integralLimit):
        if (integralLimit is not None):
            self.integralLimit = integralLimit
        
    def setFrequency(self, frequency):
        self.frequency = frequency

    def createTrackbars(self):
        if self.trackbars_window is None:
            return
        
        windowName = self.trackbars_window
        self.trackbarNames = []
        paramsNames = ["Kp", "Ki", "Kd"]
        n = len(self.setPoint)
        for i in range(3*n):
            self.trackbarNames.append(paramsNames[i//n]+ " " + str(i%n+1) )
            # Initial value calculation based on coefficients and current values
            if i // n == 0:  
                init_value = int(self.kp[i%n] * 1000 / self.trackbarCoeffs[0]) 
            elif i // n == 1:  
                init_value = int(self.ki[i%n] * 1000 / self.trackbarCoeffs[1])  
            else: 
                init_value = int(self.kd[i%n] * 1000 / self.trackbarCoeffs[2])  


            cv2.createTrackbar(self.trackbarNames[i], windowName, init_value, 1000, self.on_trackbar)


    def on_trackbar(self, val):
        paramSetters = [self.setKp, self.setKi, self.setKd]
        print("Kp : ", self.kp)
        print("Ki : ", self.ki)
        print("Kd : ", self.kd)
        n = len(self.setPoint)
        vals = []
        for i in range(3*n):
            vals.append(self.trackbarCoeffs[i//n] * cv2.getTrackbarPos(self.trackbarNames[i], self.trackbars_window) / 1000)
            if len(vals) == n:
                valsArr = np.array(vals)
                paramSetters[i//n](valsArr.reshape(n,1))
                vals = []



    def updateTrackbars(self):
            if self.trackbars_window is None:
                return
            n = len(self.setPoint)
            for i in range(3 * n):
                if i // n == 0:  
                    value = int(self.kp[i%n] * 1000 / self.trackbarCoeffs[0]) 
                elif i // n == 1:  
                    value = int(self.ki[i%n] * 1000 / self.trackbarCoeffs[1])  
                else: 
                    value = int(self.kd[i%n] * 1000 / self.trackbarCoeffs[2])  

                cv2.setTrackbarPos(self.trackbarNames[i], self.trackbars_window, value)


    #######################################




    ######### Control system methods ############

    def updateInput(self, input):
        self.input = input
        self.err = self.setPoint - self.input

    def computeCommand(self, input, dt):
        
        # Compute error
        self.updateInput(input) # In case used externally

        # Compute command (PID)
        self.command = self.kp * self.err + self.ki * self.integral + self.kd * self.derivative

        # Update integral and derivative
        self.integral += self.err * (abs(self.integral) < self.integralLimit)
        self.derivative = (self.err - self.oldErr) / (dt+1e-10)
        self.oldErr = self.err
        return self.command

    def getError(self):
        return self.err

    def onRun(self):
        self.computeCommand(self.input, self.dt) # Compute command
        self.actuator.control(self.command) # Update actuator with the new command



    def startAutoRun(self, frequency=20):
        self.frequency = frequency
        self.start()


    def stopAutoRun(self):
        self.stop()


    def onStop(self):
        self.command = np.zeros((len(self.setPoint),1))
        self.integral = np.zeros((len(self.setPoint),1))

    def getLastCommand(self):
        return self.command


    def setActuator(self, actuator):
        self.actuator = actuator

    