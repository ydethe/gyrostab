#! /bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

from libSystemControl.Loggeur import Loggeur
from libSystemControl.Simulation import ASimulation

from FiltreKalmanGyro import FiltreKalmanGyro
from PenduleGyro import PenduleGyro
from Capteur import Capteur
from Controller import Controller


def valide_modele_kal():
   t = 0.
   
   # Pas de temps
   dt = 0.1
   
   # Angle initial
   th0 = 5*np.pi/180.
   
   # Nombre de pas de temps simulés
   ns = 100
   # ns = 3
   
   sys = PenduleGyro()
   sys.set_state(np.matrix([th0, 0., np.pi/2.]).T, t)
   u = np.matrix([[-sys.A*th0/sys.K]])
   
   kal = FiltreKalmanGyro()
   kal.set_state(np.matrix([th0,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., t)
   kal.set_matrices(sys.A, sys.K, dt)
   
   log = Loggeur()
   
   for i in range(ns):
      xs,_ = sys.get_state()
      xk,_ = kal.get_estimation()
      
      log.log('t', t)
      log.log('xs', xs[0,0]*180./np.pi)
      log.log('xk', xk[0,0]*180./np.pi)
      
      t += dt
      
      kal.prediction(u)
      sys.integre_pas(u,dt)
      
   fig = plt.figure()
   
   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('t', 'xs', axe, label=u'integ')
   log.plot('t', 'xk', axe, label=u'kal')
   # log.plot('t', 'xk-xs', axe, label=u'kal-integ')
   axe.legend(loc='best')
   
   plt.show()
   
   
class Simulation (ASimulation):
   def comportement(self, x, t):
      return np.matrix([[0.*np.pi/180.]])
      
def main():
   # Pas de temps
   dt = 0.1
   
   # Angle initial
   th0 = np.pi
   
   sys = PenduleGyro()
   sys.set_state(np.matrix([th0, 0., np.pi/2.]).T, 0.)
   
   kal = FiltreKalmanGyro()
   kal.set_state(np.matrix([th0,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., 0.)
   
   kal.set_matrices(sys.A, sys.K, dt)
   
   c = Capteur()
   
   ctrl = Controller(dt, sys)
   
   sim = Simulation(dt, ctrl, sys, c, kal)
   
   sim.simule(20.)
   log = sim.get_loggeur()
   t = log.getValue('t')
   u = log.getValue('cmd')
   ps = log.getValue('ps_sim')
   dpsi = u/np.sin(ps)
   
   # =============================================
   # Tracés
   # =============================================
   fig = plt.figure()
   
   axe = fig.add_subplot(211)
   axe.grid(True)
   log.plot('t', 'ps_sim', axe, label=u'psi')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(212, sharex=axe)
   axe.grid(True)
   axe.plot(t, dpsi, label=u'dpsi')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   fig.tight_layout()
   
   
   fig = plt.figure()
   
   axe = fig.add_subplot(231)
   axe.grid(True)
   log.plot('t', 'th_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'th_sim', axe, label=u'simu')
   log.plot('t', 'th_mes', axe, label=u'mesuré', marker='+', linestyle='')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(234, sharex=axe)
   axe.grid(True)
   # log.plot('t', 'sig_th', axe, label=u'sigma th_sim')
   log.plot('t', 'th_est-th_sim', axe, label=u'delta th_sim')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   axe = fig.add_subplot(232)
   axe.grid(True)
   log.plot('t', 'dth_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'dth_sim', axe, label=u'simu')
   log.plot('t', 'dth_mes', axe, label=u'mesuré', marker='+', linestyle='')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(235, sharex=axe)
   axe.grid(True)
   # log.plot('t', 'sig_dth', axe, label=u'sigma dth_sim')
   log.plot('t', 'dth_est-dth_sim', axe, label=u'delta dth_sim')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   axe = fig.add_subplot(233)
   axe.grid(True)
   log.plot('t', 'biais_est', axe, label=u'estimé', marker='o', linestyle='')
   # log.plot('t', 'biais', axe, label=u'simu')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(236, sharex=axe)
   axe.grid(True)
   # log.plot('t', 'sig_biais', axe, label=u'sigma biais')
   # log.plot('t', 'biais_est-biais', axe, label=u'delta biais')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   fig.tight_layout()
   plt.show()
   

if __name__ == '__main__':
   main()
   # valide_modele_kal()

