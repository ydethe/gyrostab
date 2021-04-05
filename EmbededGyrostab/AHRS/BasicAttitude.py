# -*- coding: utf-8 -*-

import math
import numpy as np
import warnings

from numpy.linalg import norm

class BasicAttitude:
    
    
    def __init__(self, pitch=0.0, roll=0.0, yaw=0.0):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.magnetometer_TL = (0.0,0.0,0.0)
        self.accelerometer_TL = (0.0,0.0,0.0)
    
    def update(self, gyroscope, accelerometer, magnetometer):
        
        accelerometer = np.array(accelerometer, dtype=float).flatten()
        # Normalise accelerometer measurement
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)
        
        magnetometer = np.array(magnetometer, dtype=float).flatten()
        # Normalise accelerometer measurement
        if norm(magnetometer) is 0:
            warnings.warn("magnetometer is zero")
            return
        magnetometer /= norm(magnetometer)
        
        self.pitch = math.pi/2 - np.arctan2(np.sqrt(accelerometer[1]*accelerometer[1] + accelerometer[2]*accelerometer[2]), accelerometer[0])
        self.roll = -1 * math.pi/2 - np.arctan2(accelerometer[2], accelerometer[1])
        
        cos_p = np.cos(self.pitch)
        sin_p = np.sin(self.pitch)
        cos_r = np.cos(self.roll)
        sin_r = np.sin(self.roll)*-1.0
        m_pitch = np.array([[cos_p, 0, sin_p], [0, 1, 0], [sin_p*-1, 0, cos_p]], dtype=float)
        m_roll = np.array([[1, 0, 0], [0, cos_r, sin_r], [0, sin_r*-1, cos_r]], dtype=float)
        
        m_pass_TPF2TL = np.matmul(m_pitch, m_roll)
        self.magnetometer_TL = np.matmul(m_pass_TPF2TL, magnetometer)
        self.accelerometer_TL = np.matmul(m_pass_TPF2TL, accelerometer)

        self.yaw = np.arctan2(self.magnetometer_TL[1], self.magnetometer_TL[0])
        
        return self.roll, self.pitch, self.yaw
        
