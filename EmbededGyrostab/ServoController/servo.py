

import ServoController.pythonSB as pySB

class Servo:
    
    pinNumber = 0
    minPulse = 1000
    maxPulse = 2000
    minAngle = -90
    maxAngle = 90
    
    def __init__(self, pinNumber, minPulse=1000, maxPulse=2000, minAngle=-90, maxAngle=90):
        self.configure(pinNumber, minPulse, maxPulse, minAngle, maxAngle)
    
    def configure(self, pinNumber, minPulse, maxPulse, minAngle, maxAngle):
        self.pinNumber = pinNumber
        self.minPulse = minPulse
        self.maxPulse = maxPulse
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        pySB.servo_configure(self.pinNumber, self.minPulse, self.maxPulse, self.minAngle, self.maxAngle)
        
    def setAngle(self, angle):
        pySB.servo_set_angle(self.pinNumber, angle)
