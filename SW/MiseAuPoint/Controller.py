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
# Définition du controleur
# =============================================
class Controller (AController):
   def __init__(self, dt, sys, capteurs, estimateurs):
      estimateurs.set_matrices(sys.A, sys.K, dt)
      AController.__init__(self, 'ctrl', dt, sys, capteurs, estimateurs)
      
      self._int_th = 0.
      
   def comportement(self,cons,Xest):
      P = -0.4499
      I = -0.4760
      D = -0.2016
      
      th = Xest[0,0]
      dth1 = Xest[1,0]
      # biais = Xest[2,0]
      th_c = cons[0,0]
      
      u = P*(th-th_c) + I*self._int_th + D*dth1
      # dps1 = np.array([u/np.sin(ps)])
      self._int_th += (th-th_c)*self._dt
      
      return np.matrix([[u]])

