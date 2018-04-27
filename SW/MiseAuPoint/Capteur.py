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
# Définition des capteurs
# =============================================
class Capteur (ACapteur):
   def __init__(self):
      self.std_ang_deg = 1.
      self.std_gyr_deg = 1./10
      self.biais_gyr_deg = -5
      
      moy = np.matrix([0., self.biais_gyr_deg*np.pi/180.]).T
      cov = np.matrix((np.diag([self.std_ang_deg,self.std_gyr_deg])*np.pi/180.)**2)
      ACapteur.__init__(self, ['th_mes', 'dth_mes'], moy, cov)
      
   def comportement(self, x, t):
      return x[0:2,:]


def test():
   import numpy as np
   import matplotlib.pyplot as plt
   
   from libSystemControl.Loggeur import Loggeur

   log = Loggeur()
   
   fs = 100.
   ns = 100
   t = np.arange(ns)/fs
   
   c0 = Capteur([0], biais_mes=False)
   cb = Capteur([0], biais_mes=True)
   
   for i in range(ns):
      x = 10.*np.pi/180.*np.sin(2*np.pi*t[i])
      xmes0 = c0.mesure(np.matrix([[x]]),t[i])
      xmesb = cb.mesure(np.matrix([[x]]),t[i])
      
      log.log('x', t[i],x*180./np.pi)
      log.log('xmes0', t[i],xmes0[0,0]*180./np.pi)
      log.log('xmesb', t[i],xmesb[0,0]*180./np.pi)
      
   fig = plt.figure()
   
   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('xmesb', axe, label=u'mesuré + biais (=5)', marker='o', linestyle='')
   log.plot('xmes0', axe, label=u'mesuré', marker='+', linestyle='')
   log.plot('x', axe, label=u'simu')
   axe.legend()
   axe.set_xlabel(u"Temps (s)")
   
   plt.show()
   

if __name__ == '__main__':
   test()

