#! /bin/env python2
# -*- coding: utf-8 -*-

import sys
sys.path.append('..')

import numpy as np
from numpy import cos, sin, cosh, sinh, sqrt, exp
import matplotlib.pyplot as plt

from libSystemControl.Estimateur import FiltreKalman
from libSystemControl.Simulation import ASimulation

from SystemeTest import SystemeTest
from Capteur import Capteur
from Controller import Controller
   
 

class Simulation (ASimulation):
   def comportement(self, x, t):
      return np.matrix([[1.]])
      
      
def main():
   # Cadence de calcul
   fs = 100.
   
   # Init système
   sys = SystemeTest(1./fs)
   sys.set_state(np.matrix([2.,0.]).T, 0.)
   
   # Init Kalman
   w = np.sqrt(-sys.A)/fs
   F = np.matrix([[ cos(w),      sin(w)/(w*fs)],
                  [-w*fs*sin(w), cos(w)       ]])
   B = np.matrix([sys.K/(w*fs)**2*(1. - cos(w)), sys.K/(w*fs)*sin(w)]).T
   H = np.matrix([[1., 0.]])
   Q = np.matrix(np.eye(2))*0
   std_mes = 0.1
   R = np.matrix(np.eye(1))*std_mes**2
   
   kal = FiltreKalman(['pos_est', 'vel_est'])
   kal.set_matrices(F, B, H, Q, R, 1./fs)
   kal.set_state(np.matrix([2.,0.]).T, np.matrix(np.diag((1.,1.)))/10., 0.)
   
   c = Capteur()
   
   ctrl = Controller(1./fs, sys, c, kal)
   
   sim = Simulation(1./fs, ctrl, sys, c, kal)
   
   sim.simule(20.)
   log = sim.get_loggeur()
   
   # =============================================
   # Tracés
   # =============================================
   fig = plt.figure()
   
   t = log.getValue('t')
   
   axe = fig.add_subplot(221)
   axe.grid(True)
   log.plot('t', 'pos_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'pos_sim', axe, marker='*', label=u'simu', linestyle='')
   log.plot('t', 'pos_mes', axe, label=u'mesuré', marker='+', linestyle='')
   # axe.plot(t, 2*cos(t*2*np.pi), label=u'exact')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(223, sharex=axe)
   axe.grid(True)
   # log.plot('t', 'sig_th', axe, label=u'sigma th')
   log.plot('t', 'pos_est-pos_sim', axe, label=u'delta th')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   axe = fig.add_subplot(222)
   axe.grid(True)
   log.plot('t', 'vel_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'vel_sim', axe, label=u'simu')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(224, sharex=axe)
   axe.grid(True)
   # log.plot('t', 'sig_dth', axe, label=u'sigma dth')
   log.plot('t', 'vel_est-vel_sim', axe, label=u'delta dth')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   fig.tight_layout()
   plt.show()
   

if __name__ == '__main__':
   main()

