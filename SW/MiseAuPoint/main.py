#! /bin/env python2
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use('tkagg')

import numpy as np
import matplotlib.pyplot as plt

from libSystemControl.Loggeur import Loggeur

from FiltreKalmanGyro import FiltreKalmanGyro
from PenduleGyro import PenduleGyro
from Capteur import Capteur
from Controller import Controller


def main():
   BIAIS_MES = True
   
   kal = FiltreKalmanGyro(BIAIS_MES)
   
   sys = PenduleGyro()
   
   c = Capteur([0], BIAIS_MES)
   
   dt = 1./50.
   ctrl = Controller(dt, sys, c, kal)
   
   # =============================================
   # Simulation
   # =============================================
   log = Loggeur()
   
   th0 = 1.*np.pi/180.
   dt = 1./50.
   th_c = np.matrix([[10.*np.pi/180.]])
   ns = 500
   x = np.matrix([th0, 0., np.pi/2.]).T
   t = 0.
   sys.set_state(x, t)
   if BIAIS_MES:
      kal.set_state(np.matrix([th0,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., t)
   else:
      kal.set_state(np.matrix([th0,0.]).T, np.matrix(np.diag((1.,1.)))/10., t)
   int_th = 0.
   for i in range(ns):
      ctrl.integre_pas(th_c)
      
      x,t = kal.get_estimation()
      th_est  = x[0,0]
      dth_est = x[1,0]
      if BIAIS_MES:
         biais_est  = x[2,0]
         log.log('biais_est', t,biais_est*180./np.pi)
      
      x,t = sys.get_state()
      theta  = x[0,0]
      dtheta = x[1,0]
      psi    = x[2,0]
      
      u = ctrl.get_last_consigne()
      dpsi = u[0,0]/np.sin(psi)
      
      mes = ctrl.get_last_measurement()
      
      log.log('theta_mes', t,mes[0,0]*180./np.pi)
      log.log('theta_est', t,th_est*180./np.pi)
      log.log('theta', t,theta*180./np.pi)
      log.log('dtheta_est', t,dth_est*180./np.pi)
      log.log('dtheta', t,dtheta*180./np.pi)
      log.log('psi', t,psi*180./np.pi)
      
      
   # =============================================
   # Tracés
   # =============================================
   fig = plt.figure()
   
   axe = fig.add_subplot(211)
   axe.grid(True)
   log.plot('psi', axe, label=u'simu')
   axe.legend()
   axe.set_title(u"Angle de précession (deg)")
   
   axe = fig.add_subplot(212, sharex=axe)
   # axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('theta_mes', axe, label=u'mesuré', marker='+', linestyle='')
   log.plot('theta', axe, label=u'simu')
   log.plot('theta_est', axe, label=u'estimé')
   axe.legend()
   axe.set_title(u"Angle du pendule (deg)")
   axe.set_xlabel(u"Temps (s)")
   
   fig.tight_layout()
   plt.show()
   

if __name__ == '__main__':
   main()

