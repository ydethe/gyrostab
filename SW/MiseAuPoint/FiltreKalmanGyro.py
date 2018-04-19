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
# Définition de filtre de Kalman
# =============================================
class FiltreKalmanGyro (FiltreKalman):
   def __init__(self, biais_mes=False):
      FiltreKalman.__init__(self, 'kalman')
      self._biais_mes = biais_mes
      
   def set_matrices(self, A, K, dt):
      self._A = A
      self._K = K
      self._dt = dt
      
      w = np.sqrt(A)*dt
      
      if self._biais_mes:
         self._F = np.matrix([[np.cosh(w),         np.sinh(w)/sqrt(A), 0.],
                              [sqrt(A)*np.sinh(w), np.cosh(w),         0.],
                              [0.,                 0.,                 1.]])
         self._B = np.matrix(np.zeros((3,1)))
         self._H = np.matrix([[1., 0., 1.]])
         self._Q = np.matrix(np.eye(3))*1.e-4
      
      else:
         self._F = np.matrix([[ np.cosh(w),         np.sinh(w)/sqrt(A)],
                              [sqrt(A)*np.sinh(w), np.cosh(w),       ]])
         self._B = np.matrix(np.zeros((2,1)))
         self._H = np.matrix([[1., 0.]])
         self._Q = np.matrix(np.eye(2))*0
      
      self._B[0,0] = K/A*(np.cosh(w) - 1.)
      self._B[1,0] = K/sqrt(A)*np.sinh(w)
      
      self._std_mes = np.pi/180*10
      self._R = np.matrix(np.eye(1))*self._std_mes**2
      
   def prediction(self, u):
      FiltreKalman.prediction(self, u)
      
      
def test():
   import matplotlib
   matplotlib.use('tkagg')

   import numpy as np
   import matplotlib.pyplot as plt
   
   from libSystemControl.Loggeur import Loggeur

   log = Loggeur()
   
   fs = 100.
   ns = 1000
   A = (2*np.pi)**2
   K = 1.
   w = np.sqrt(A)/fs
   
   # Init système
   th = 10.
   dth = 0.
   
   # Init Kalman
   kal = FiltreKalmanGyro(biais_mes=False)
   kal.set_matrices(A, K, 1./fs)
   kal.set_state(np.matrix([0.,0.]).T, np.matrix(np.diag((1.,1.)))/10., 0.)
   
   x_cons = 5.
   
   Xint = 0.
   for i in range(ns):
      # MAJ commande
      Xest,t = kal.get_estimation()
      a = 5.
      P = (3*a**2+A)/K
      I = a**3/K
      D = 3*a/K
      u = -(P*(Xest[0,:]-x_cons) + I*Xint + D*Xest[1,:])
      
      # MAJ système
      th2  = np.cosh(w)*th + np.sinh(w)/sqrt(A)*dth + K/A*(np.cosh(w) - 1.)*u[0,0]
      dth2 = sqrt(A)*np.sinh(w)*th + np.cosh(w)*dth + K/sqrt(A)*np.sinh(w)*u[0,0]
      th = th2
      dth = dth2
      
      # Mesure
      mes = np.matrix([th + np.random.normal()/2])
      
      # MAJ Kalman
      kal.comportement(mes, u, t)
      
      # MAJ Loggeur
      x,t = kal.get_estimation()
      th_est  = x[0,0]
      dth_est = x[1,0]
      
      Xint += (th_est-x_cons)/fs
      
      log.log('th_mes', t,mes[0,0])
      log.log('th', t,th)
      log.log('dth', t,dth)
      log.log('th_est', t,th_est)
      log.log('dth_est', t,dth_est)
      
   fig = plt.figure()
   
   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('th_mes', axe, label=u'mesuré', marker='+', linestyle='')
   log.plot('th_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('th', axe, label=u'simu')
   axe.legend()
   axe.set_xlabel(u"Temps (s)")
   
   plt.show()
   

if __name__ == '__main__':
   test()

