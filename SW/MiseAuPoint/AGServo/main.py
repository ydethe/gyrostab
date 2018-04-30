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
      return np.matrix([[np.pi/180.*20]])
      
      
def main():
   # Cadence de calcul
   fs = 100.
   
   # Init système
   lbda = 5.
   sys = SystemeTest(lbda)
   sys.set_state(np.matrix([-np.pi/4]).T, 0.)
   
   # Init Kalman
   A = np.matrix([[ np.exp(-lbda/fs), 0., 0.],
                  [ 0.,    1., 0.],
                  [ 0.,    0., 1.]])
   B = np.matrix([1. - np.exp(-lbda/fs), 0., 0.]).T
   C = np.matrix([[1.,    1., 0.],
                  [-lbda, 0., 1.]])
   D = np.matrix([[0., lbda]]).T
   Q = np.matrix(np.eye(3))*0
   std_pos = 0.00448355705097
   std_vel = 0.0014239412603
   R = np.matrix((np.diag([std_pos, std_vel]))**2)
   
   kal = FiltreKalman(['pos_est','biais_pos_est','biais_vel_est'])
   kal.set_matrices(A,B,C,D, Q,R, 1./fs)
   kal.set_state(np.matrix([0.,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., 0.)
   
   c = Capteur(lbda)
   
   ctrl = Controller(1./fs, sys)
   
   sim = Simulation(1./fs, ctrl, sys, c, kal)
   
   sim.simule(20.)
   log = sim.get_loggeur()
   
   # =============================================
   # Tracés
   # =============================================
   fig = plt.figure()
   
   t = log.getValue('t')
   
   axe = fig.add_subplot(211)
   axe.grid(True)
   log.plot('t', 'pos_est*180/np.pi', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'pos_sim*180/np.pi', axe, label=u'simu')
   # log.plot('t', '(pos_est-pos_sim)*180/np.pi', axe, label=u'delta th')
   log.plot('t', 'pos_mes*180/np.pi', axe, label=u'angle mesuré', marker='+', linestyle='')
   axe.legend(loc='best')
   
   axe = fig.add_subplot(212, sharex=axe)
   axe.grid(True)
   # log.plot('t', 'ctrl*180/np.pi', axe, label=u'Commande')
   # log.plot('t', 'vel_mes*180/np.pi', axe, label=u'vitesse mesurée', marker='+', linestyle='')
   log.plot('t', 'biais_pos_est*180/np.pi', axe, label=u'biais pos')
   log.plot('t', 'biais_vel_est*180/np.pi', axe, label=u'biais vel')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   fig.tight_layout()
   plt.show()
   

if __name__ == '__main__':
   main()

