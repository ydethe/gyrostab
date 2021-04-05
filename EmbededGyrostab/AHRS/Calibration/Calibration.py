
import numpy as np
from numpy.linalg import norm

import math, random, queue

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class SphericalCalibration:
  
    def __init__(self, scaleFactor = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                       bias = np.array([0.0, 0.0, 0.0]), filterSize = 10):
        self.scaleFactor = scaleFactor
        self.bias = bias
        self.sphere = []
        self.filterSize = filterSize
        self.fibonacci_sphere(20, False)
        #fig = plt.figure()
        #self.ax = fig.add_subplot(111, projection='3d')
        #self.ax.set_aspect('equal')
        #plt.ion()
        #plt.show()
        #self.plotSphere()
    
    def __str__(self):
        return str(self.sphere)
    
    def status(self):
        status =  []
        for point in self.sphere:
            status.append(len(point['fifo']))
        return status
    
    def isCalibrated(self):
        isCalibrated = True
        for point in self.sphere:
            isCalibrated = point['fifo'].isFull
        return isCalibrated
    
    def reset():
        for point in self.sphere:
            point['fifo'].reset()
    
    def pushData(self, vector):
        maxVal = 0.0
        try:
            maxVal = float('-inf')
        except:  # check for a particular exception here?
            maxVal = -1e30000
        maxIdx = -1
        # Normlalize input vector
        normVect = vector/norm(vector)
        for idx, point in enumerate(self.sphere):
            val = np.dot(np.matrix([point['point']]), normVect)
            if val > maxVal:
                maxVal = val
                maxIdx = idx
        if maxIdx != -1:
            alreadyFull = self.sphere[maxIdx]['fifo'].isFull  
            self.sphere[maxIdx]['fifo'].put(vector)
        
            #if self.sphere[maxIdx]['fifo'].isFull and not alreadyFull:
                #self.ax.scatter(self.sphere[maxIdx]['point'][0], self.sphere[maxIdx]['point'][1], self.sphere[maxIdx]['point'][2], c='b')
                #plt.draw()
                #plt.pause(0.001)
                #print(str(maxIdx) + "th fifo is full")
    
    def fibonacci_sphere(self, samples=1,randomize=True):
        rnd = 1.
        if randomize:
            rnd = random.random() * samples

        offset = 2./samples
        increment = math.pi * (3. - math.sqrt(5.));

        for i in range(samples):
            y = ((i * offset) - 1) + (offset / 2);
            r = math.sqrt(1 - pow(y,2))

            phi = ((i + rnd) % samples) * increment

            x = math.cos(phi) * r
            z = math.sin(phi) * r
            
            self.sphere.append({'point': [x,y,z], 'fifo': Filter(self.filterSize)})

    def plotSphere(self):
        for point in self.sphere:
            self.ax.scatter(point['point'][0], point['point'][1], point['point'][2], c='r')
            plt.draw()
        plt.pause(0.001)

class LinearCalibration:
    
    def __init__(self, scaleFactor = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                   bias = np.array([0.0, 0.0, 0.0]), filterSize = 10):
        self.filter = Filter(filterSize)
        self.scaleFactor = scaleFactor
        self.bias = bias
    
    def reset(self):
        self.filter.reset()
    
    def pushData(self, vector):
        self.filter.put(vector)
        self.bias = self.filter.mean().flatten()
        
    def status(self):
        return len(self.filter)
    
    def isCalibrated(self):
        return self.filter.isFull
    

class Filter:
   
    def __init__(self, maxSize=1):
        self.maxSize = maxSize
        self.fifo = np.zeros((1,3), dtype=float)
        self.currentIndex=0
        self.isFull = False
    
    def __str__(self):
        return str(self.fifo)
    
    def __len__(self):
        return len(self.fifo)
    
    def reset(self):
        self.fifo = np.zeros((1,3), dtype=float)
        self.currentIndex=0
        self.isFull = False
    
    def put(self, vector):
        if not self.isFull: # Resize the FIFO until it reach its maximum size
            self.fifo.resize((self.currentIndex+1,3), refcheck=False)
        # Update the older vector by the new one
        self.fifo[self.currentIndex] = vector
        # Loop on matrix line
        self.currentIndex = (self.currentIndex + 1) % self.maxSize
        if self.currentIndex == 0:
            self.isFull = True
    
    # Return a vector containing the mean of each matrix row
    def mean(self):
        return self.fifo.mean(0)

if __name__ == '__main__':
    cal = Calibration()
    cal.fibonacci_sphere(20, False)
    #cal.plotSphere()
    
    