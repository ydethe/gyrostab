#! /bin/env python2
# -*- coding: utf-8 -*-

import collections

import numpy as np
from numpy import cos, sin, sqrt, dot
import scipy.linalg as lin

from libSystemControl.Systeme import ASysteme
from libSystemControl.Capteur import ACapteur
from libSystemControl.Controller import AController
from libSystemControl.Estimateur import FiltreKalman


# =============================================
# Définition du système
# =============================================
class SystemeTest (ASysteme):
   def __init__(self, dt):
      ASysteme.__init__(self, [u'pos_sim', u'vel_sim'])
      
      self.A = -(2*np.pi)**2
      self.K = 1.
      self.dt = dt
      
   def jacobien(self, x, u, t):
      T, dT = x
      return np.matrix([
         [0,	1],
         [self.A, 0.]]
      )
   
   def comportement(self, x,u,t):
      th = x[0]
      dth1 = x[1]
      
      dth2 = self.A*th + self.K*u[0,0]
      
      return np.array([dth1,dth2])
      
      
      