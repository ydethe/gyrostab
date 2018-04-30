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
   def __init__(self, lbda):
      ASysteme.__init__(self, [u'pos_sim'])
      
      self.A = -lbda
      self.K = lbda
      
   def jacobien(self, x, u, t):
      return np.matrix([[self.A]])
      
   def comportement(self, x,u,t):
      th = x[0]
      
      dth = self.A*th + self.K*u[0,0]
      
      return np.array([dth])
      
      
      