

from ServoController.servo import Servo

class ServoController:
    
    servos = {}
    
    def __init__(self, servoXPin=11, servoYPin=13, servoZPin=14):
        self.servos["x"] = Servo(servoXPin, 1000, 2000, -90, 90)
        self.servos["y"] = Servo(servoYPin, 1000, 2000, -90, 90)
        self.servos["z"] = Servo(servoZPin, 1000, 2000, -90, 90)

    def setServoAngle(self, servoAxis, angle):
        self.servos[servoAxis].setAngle(angle)
        
    def setAllServoAngle(self, xAngle, yAngle, zAngle):
        self.setServoAngle("x", xAngle)
        self.setServoAngle("y", yAngle)
        self.setServoAngle("z", zAngle)
        
    def configureServo(self, servoAxis, servoPin, minPulse, maxPulse, minAngle, maxAngle):
        self.servos[servoAxis].configure(servoPin, minPulse, maxPulse, minAngle, maxAngle)
        
    