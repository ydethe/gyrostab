# -*- coding: utf-8 -*-

import math
import threading
import time

import numpy as np

from AHRS.Adafruit_LSM9DS0 import LSM9DS0
import AHRS.BasicAttitude as BasicAttitude
import AHRS.Calibration.Calibration as Calibration

class AHRS(threading.Thread):
    
    def __init__(self, period=1/10):
        threading.Thread.__init__(self)
        self.imuDevice = LSM9DS0() # Use for IMU acquisition
        
        self.accelerometer = np.array([0.0, 0.0, 0.0])
        self.gyroscope = np.array([0.0, 0.0, 0.0])
        self.magnetometer = np.array([0.0, 0.0, 0.0])
        self.acqisitionTime = 0.0
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.attitudeTime = 0.0
        
        self.periodUpdateRun = True
        self.next_call = 0.0
        self.period = period
        
        self.attitudeAlgorithm = BasicAttitude.BasicAttitude()

        self.gyro_cal = Calibration.LinearCalibration(filterSize=100)

        self.accel_cal = Calibration.SphericalCalibration()
        
        self.updateIMU()

        
    def run(self):
        self.next_call = time.time()
        self.periodicUpdate()
        print("Stopping AHRS thread")
        
    def stop(self):
        self.periodUpdateRun=False
    
    def periodicUpdate(self):
        if self.periodUpdateRun:
            self.update()
            self.next_call = self.next_call + self.period
            timeToWait = self.next_call - time.time()
            if timeToWait < 0:
                print("AHRS expired deadline: " + str(timeToWait))
            threading.Timer( timeToWait, self.periodicUpdate ).start()
        else:
            print("Stopping AHRS periodic update")

    def update(self):
        self.updateIMU()
        self.updateAttitude()

    def updateAttitude(self):
        self.attitudeTime = time.time()
        self.roll, self.pitch, self.yaw = self.attitudeAlgorithm.update(gyroscope = self.gyroscope,
                                                                        accelerometer = self.accelerometer,
                                                                        magnetometer = self.magnetometer)
        
     # Update IMU value to following attribute : accelerometer, magnetometer, gyroscope
    def updateIMU(self):        
        # IMU acquisition - MKSI Unit
        self.acqisitionTime = time.time()
        gyroscope, magnetometer, accelerometer = self.imuDevice.read()
        
        self.gyroscope = np.array(gyroscope)
        self.magnetometer = np.array(magnetometer)
        self.accelerometer = np.array(accelerometer)
        
        # Push data if gyro is not calibrated yet
        if not self.gyro_cal.isCalibrated():
            self.gyro_cal.pushData(self.gyroscope)
        # Apply bias from claibration on gyro
        self.gyroscope = self.gyroscope - self.gyro_cal.bias
        
        # Push data if accel is not calibrated yet
        if not self.accel_cal.isCalibrated():
                self.accel_cal.pushData(self.accelerometer)
        # Apply bias from claibration on gyro
        self.accelerometer = self.accelerometer - self.accel_cal.bias

    def calibrateGyrometer(self):
        self.gyro_cal.reset()
        
    def calibrateAccelerometer(self):
        self.gyro_cal.reset()

    def getIMUData(self):
        return {'time': self.acqisitionTime,
               'acceleration': self.accelerometer.flatten().tolist(),
               'angularSpeed': self.gyroscope.flatten().tolist(),
               'magneticField': self.magnetometer.flatten().tolist()}
    
    def getDisplayableAcceleration(self):
        return [{'time': (int)(self.acqisitionTime), 'y': self.accelerometer.flatten()[0]},
                {'time': (int)(self.acqisitionTime), 'y': self.accelerometer.flatten()[1]},
                {'time': (int)(self.acqisitionTime), 'y': self.accelerometer.flatten()[2]}]
    
    def getDisplayableAccelerationCaption(self):
        return {'chartTitle': "Acceleration", 'curvesTitle': ["X axis (g)", "Y axis (g)", "Z axis (g)"]}
    
    def getDisplayableAngularSpeed(self):
        return [{'time': (int)(self.acqisitionTime), 'y': self.gyroscope.flatten()[0]*180/math.pi},
                {'time': (int)(self.acqisitionTime), 'y': self.gyroscope.flatten()[1]*180/math.pi},
                {'time': (int)(self.acqisitionTime), 'y': self.gyroscope.flatten()[2]*180/math.pi}]
    
    def getDisplayableAngularSpeedCaption(self):
        return {'chartTitle': "Angular speed", 'curvesTitle': ["X axis (°/s)", "Y axis (°/s)", "Z axis (°/s)"]}
    
    def getDisplayableMagneticField(self):
        return [{'time': (int)(self.acqisitionTime), 'y': self.magnetometer.flatten()[0]},
                {'time': (int)(self.acqisitionTime), 'y': self.magnetometer.flatten()[1]},
                {'time': (int)(self.acqisitionTime), 'y': self.magnetometer.flatten()[2]}]
    
    def getDisplayableMagneticFieldCaption(self):
        return {'chartTitle': "Magnetic field", 'curvesTitle': ["X axis (gauss)", "Y axis (gauss)", "Z axis (gauss)"]}
    
    def getAttitudeData(self):
        return {'time': self.attitudeTime, 'attitude': [self.roll, self.pitch, self.yaw]}
    
    def getDisplayableEuler(self):
        return [{'time': (int)(self.attitudeTime), 'y': self.roll*180/math.pi}, # Roll (°)
                {'time': (int)(self.attitudeTime), 'y': self.pitch*180/math.pi}, # Pitch (°)
                {'time': (int)(self.attitudeTime), 'y': self.yaw*180/math.pi}] # Yaw (°)attitude
    
    def getDisplayableEulerCaption(self):
        return {'chartTitle': "Attitude", 'curvesTitle': ["Roll (°)", "Pitch (°)", "Yaw (°)"]}
    
    def getDisplayableMagnetometerTL(self):
        return [{'time': (int)(self.attitudeTime), 'y': self.attitudeAlgorithm.magnetometer_TL[0]},
                {'time': (int)(self.attitudeTime), 'y': self.attitudeAlgorithm.magnetometer_TL[1]},
                {'time': (int)(self.attitudeTime), 'y': self.attitudeAlgorithm.magnetometer_TL[2]}]
    
    def getDisplayableMagnetometerTLCaption(self):
        return {'chartTitle': "Inertial magnetic field", 'curvesTitle': ["X axis (gauss)", "Y axis (gauss)", "Z axis (gauss)"]}
    
    def getDisplayableAccelerometerTL(self):
        return [{'time': (int)(self.attitudeTime), 'y': self.attitudeAlgorithm.accelerometer_TL[0]},
                {'time': (int)(self.attitudeTime), 'y': self.attitudeAlgorithm.accelerometer_TL[1]},
                {'time': (int)(self.attitudeTime), 'y': self.attitudeAlgorithm.accelerometer_TL[2]}]
    
    def getDisplayableAccelerationTLCaption(self):
        return {'chartTitle': "Intiertial acceleration", 'curvesTitle': ["X axis (g)", "Y axis (g)", "Z axis (g)"]}
    
    def getGyrometerCalibrationStatus(self):
        return {'status': self.gyro_cal.isCalibrated()}
    
    def getAccelerometerCalibrationStatus(self):
        return {'status': self.accel_cal.isCalibrated()}
    