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
   def __init__(self, dt, sys):
      AController.__init__(self, ['ctrl'], dt)
      
      a = 2.
      self._P = (sys.A-2*a)/sys.K
      self._I = -a**2/sys.K
      print u'Coefficients P, I : ', self._P,self._I
      
      self._int_th = 0.
      
   def comportement(self,cons,Xest):
      th = Xest[0,0]
      th_c = cons[0,0]
      
      # print th-th_c, self._int_th, dth1
      u = self._P*(th-th_c) + self._I*self._int_th
      self._int_th += (th-th_c)*self._dt
      
      return np.matrix([[u]])

