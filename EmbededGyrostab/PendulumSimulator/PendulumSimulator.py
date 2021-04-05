import math
import numpy as np
import time
import threading

import PendulumSimulator.PenduleGyro as pendule

class PendulumSimulator(threading.Thread):
    
    sys = pendule.PenduleGyro()
    
    simStep = 0.0
    next_call = 0.0
    periodUpdateRun = True
    
    actuatorAngle = 0.0
    
    def __init__(self, simStep=1/10, sysAngle=math.pi/4, sysAngularSpeed=0.0, actAngle=np.pi/2.):
        threading.Thread.__init__(self)
        self.simStep = simStep
        self.actuatorAngle = actAngle
        self.sys.set_state(np.matrix([sysAngle, sysAngularSpeed, actAngle]).T, 0.) # [angle initial, vitesse angulaire, angle de précession], Temps
        print("Simulation pendule init done")
        
    def run(self):
        self.next_call = time.time()
        self.periodicUpdate()
        print("Stopping Pendule simulation thread")
    
    def stop(self):
        self.periodUpdateRun=False
        
    def periodicUpdate(self):
        if self.periodUpdateRun:
            self.sys.integre_pas(np.matrix([[self.actuatorAngle]]), self.simStep)
            self.next_call = self.next_call + self.simStep
            threading.Timer( self.next_call - time.time(), self.periodicUpdate ).start()
        else:
            print("Stopping Pendule simulation periodic update")
    
    def setActuatorAngle(self, angle):
        self.actuatorAngle = angle
    
    def getState(self):
        state, simTime = self.sys.get_state()
        return {'time': simTime, 'state': state}
    
    def getDisplayableState(self):
        state, simTime = self.sys.get_state()
        return [{'time': (int)(simTime), 'y': state[0,0]*180/math.pi}, # System angle (°)
                {'time': (int)(simTime), 'y': state[1,0]*180/math.pi}, # System angular speed (°/s)
                {'time': (int)(simTime), 'y': state[2,0]*180/math.pi}] # Actuator angle (°)
    
    def getDisplayableStateCaption(self):
        return {'chartTitle': "Pendule simulation", 'curvesTitle': ["Pendule angle (°)", "Pendule angular speed (°/s)", "Actuator angle (°)"]}