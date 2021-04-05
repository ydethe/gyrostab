import threading
import time
import math

from AHRS import AHRS
from ServoController import ServoController

class Controller(threading.Thread):
    
    def __init__(self, AHRS, servoController, period=1/10, consigne=0.0):
        threading.Thread.__init__(self)
        self.AHRS = AHRS
        self.servoController = servoController
        
        self.command = 0.0
        self.consigne = consigne
        
        self.periodUpdateRun = True
        self.next_call = 0.0
        self.period = period
        
    def run(self):
        self.next_call = time.time()
        self.periodicUpdate()
        print("Stopping Controller thread")
        
    def stop(self):
        self.periodUpdateRun=False
    
    def periodicUpdate(self):
        if self.periodUpdateRun:
            self.updateCommand()
            self.servoController.setServoAngle("x", self.command)
            self.next_call = self.next_call + self.period
            timeToWait = self.next_call - time.time()
            if timeToWait < 0:
                print("Controller expired deadline: " + str(timeToWait))
            threading.Timer(timeToWait, self.periodicUpdate ).start()
        else:
            print("Stopping Controller periodic update")
            
    def updateCommand(self):
        err = self.consigne - self.AHRS.pitch*180/math.pi
        self.command += err * 0.3
