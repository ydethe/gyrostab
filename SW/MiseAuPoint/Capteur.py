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
   def __init__(self, iok, biais_mes=False):
      n = len(iok)
      self._iok = iok
      ret = 0
      
      self._biais_mes = biais_mes

      if self._biais_mes:
         bias_mes_deg = 5.
      else:
         bias_mes_deg = 0.
      
      std_mes_deg = 1.
      
      moy = np.matrix([bias_mes_deg*np.pi/180.]*n).T
      cov = np.matrix(np.diag([(std_mes_deg*np.pi/180.)**2]*n))
      ACapteur.__init__(self, u'capteur%i' % n, moy, cov)
      
      # Introduction d'un retard dans la valeur mesuree de 5 pas de temps
      self._buf_mes = collections.deque(maxlen=ret+1)
      
   def comportement(self, x, t):
      self._buf_mes.append(x[self._iok,:])
      return self._buf_mes[0]


def test():
   import matplotlib
   matplotlib.use('tkagg')

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

