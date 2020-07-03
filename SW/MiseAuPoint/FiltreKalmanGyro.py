#! /bin/env python2
# -*- coding: utf-8 -*-

import collections

import numpy as np
from numpy import cos, sin, sqrt, dot
import scipy.linalg as lin

from SystemControl.Estimator import AKalmanFilter


# =============================================
# Définition de filtre de Kalman
# =============================================
class FiltreKalmanGyro (AKalmanFilter):
    def __init__(self):
        AKalmanFilter.__init__(self, 'kal', name_of_outputs=['th_mes_est', 'dth_mes_est'], name_of_states=['th_est', 'dth_est', 'biais_est'])
        self.createParameter('KA',0)
        self.createParameter('KU',0)
        self.createParameter('std_ang',1.*np.pi/180.)
        self.createParameter('std_gyr',1./10*np.pi/180)
        
    def A(self, t1: float, t2: float) -> np.array:
        """(n x n) State (or system) matrix

        Args:
          t1
            Current date (s)
          t2
            Date of the new estimation (s)

        """
        dt = t2-t1
        w = np.sqrt(self.KA)*dt
        A = np.array([[np.cosh(w),         np.sinh(w)/sqrt(self.KA), 0.],
                      [sqrt(self.KA)*np.sinh(w), np.cosh(w),         0.],
                      [0.,                 0.,                 1.]])
        return A

    def B(self, t1: float, t2: float) -> np.array:
        """(n x m) Input matrix

        Args:
          t1
            Current date (s)
          t2
            Date of the new estimation (s)

        """
        dt = t2-t1
        w = np.sqrt(self.KA)*dt
        B = np.zeros((3,1))
        B[0,0] = self.KU/self.KA*(np.cosh(w) - 1.)
        B[1,0] = self.KU/sqrt(self.KA)*np.sinh(w)
        return B
        
    def C(self, t: float) -> np.array:
        """(p x n) Output matrix

        Args:
          t
            Current date (s)

        """
        C = np.array([[1., 0., 0.],
                      [0., 1., 1.]])
        return C

    def D(self, t: float) -> np.array:
        """(p x m) Feedthrough (or feedforward) matrix

        Args:
          t
            Current date (s)

        """
        D = np.array([[0.],
                      [0.]])
        return D

    def Q(self, t: float) -> np.array:
        """(n x n) Gaussian noise covariance for the state vector

        Args:
          t
            Current date (s)

        """
        Q = np.eye(3)*1.e-4
        return Q

    def R(self, t: float) -> np.array:
        """(n x n) Gaussian noise covariance for the measurement vector

        Args:
          t
            Current date (s)

        """
        R = np.diag([self.std_ang,self.std_gyr])**2
        return R
      
      
