#! /bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from libSystemControl.Simulation import ASimulation
from libSystemControl.Controller import NullController, PIDController

from AGPendule.FiltreKalmanGyro import FiltreKalmanGyro
from AGPendule.PenduleGyro import PenduleGyro
from AGPendule.Capteur import Capteur


__all__ = ['main']

def valide_modele_kal():
   from libSystemControl.Logger import Logger
   
   t = 0.

   # Pas de temps
   dt = 0.1

   # Angle initial
   th0 = 5*np.pi/180.

   # Nombre de pas de temps simulés
   ns = 100

   sys = PenduleGyro()
   sys.setState(np.matrix([th0, 0., np.pi/2.]).T, t)
   u = np.matrix([[-sys.A*th0/sys.K]])
   
   kal = FiltreKalmanGyro()
   kal.setEstimation(np.matrix([th0,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., t)
   kal.setMatrices(sys.A, sys.K, dt)
   
   log = Logger()

   for i in range(ns):
      xs,_ = sys.getState()
      xk,_ = kal.getEstimation()

      log.log('t', t)
      log.log('xs', xs[0,0]*180./np.pi)
      log.log('xk', xk[0,0]*180./np.pi)
      
      t += dt
      
      # On court-circuite l'étape de mise à jour :
      # on veut valider l'étape de prediction seule
      xest_pred,P_pred = kal._prediction(kal._xest, kal._P, u)
      kal._xest = xest_pred
      kal._P = P_pred
      kal._t = t
      # Fin du court-circuitage
      
      sys.updateSystem(u,dt)

   fig = plt.figure()

   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('t', 'xs', axe, label=u'integ')
   log.plot('t', 'xk', axe, label=u'kal')
   # log.plot('t', 'xk-xs', axe, label=u'kal-integ')
   axe.legend(loc='best')

   plt.show()


class Simulation (ASimulation):
   def behavior(self, x, t):
      return np.matrix([[0.*np.pi/180.]])
      # return np.matrix([[5.*np.pi/180.]])
      # return np.matrix([[10.*t/60.*np.pi/180.]])

def main():
   # Pas de temps
   dt = 0.01

   # Angle initial
   th0 = np.pi/180*(170)

   sys = PenduleGyro(method='dop853')
   sys.setState(np.matrix([th0, 0., np.pi/2.]).T, 0.)

   kal = FiltreKalmanGyro()
   kal.setEstimation(np.matrix([np.pi,0.,0.]).T, np.matrix(np.diag((0.,0.,0.1))), 0.)
   
   kal.setMatrices(sys.A, sys.K, dt)

   c = Capteur()
   
   a = 7.
   P = -(3*a**2+sys.A)/sys.K
   I = -a**3/sys.K
   D = -3*a/sys.K
   # P = 687.61701433/1000
   # D = 3.14576211/1000
   # I = 0.
   ctrl = PIDController(['cmd'], dt, P, I, D)
   # ctrl = NullController(['cmd'])

   sim = Simulation(dt, ctrl, sys, c, kal)
   # log = sim.getLoggeur()
   # log.setFile('log.txt')

   sim.simulate(30.)
   log = sim.getLogger()
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
   log.plot('t', 'ps_sim*180./np.pi', axe, label=u'psi')
   axe.set_title("ps_sim (deg)")
   axe.legend(loc='best')

   axe = fig.add_subplot(212, sharex=axe)
   axe.grid(True)
   axe.plot(t, dpsi*180./np.pi, label=u'dpsi')
   axe.set_title("dps_sim (deg/s)")
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")

   fig.tight_layout()


   fig = plt.figure()

   axe = fig.add_subplot(231)
   axe.grid(True)
   log.plot('t', 'th_est*180/np.pi', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'th_sim*180/np.pi', axe, label=u'simu')
   log.plot('t', 'th_mes*180/np.pi', axe, label=u'mesuré', marker='+', linestyle='')
   axe.set_title("th (deg)")
   axe.legend(loc='best')

   axe = fig.add_subplot(234, sharex=axe)
   axe.grid(True)
   log.plot('t', 'sig_th_est*180/np.pi', axe, label=u'sigma th_sim', color='green')
   log.plot('t', '-sig_th_est*180/np.pi', axe, color='green')
   log.plot('t', '(th_est-th_sim)*180/np.pi', axe, label=u'delta th_sim')
   axe.set_title("Err. est. th (deg)")
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")

   axe = fig.add_subplot(232)
   axe.grid(True)
   log.plot('t', 'dth_est*180/np.pi', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 'dth_sim*180/np.pi', axe, label=u'simu')
   log.plot('t', 'dth_mes*180/np.pi', axe, label=u'mesuré', marker='+', linestyle='')
   axe.set_title("dth (deg/s)")
   axe.legend(loc='best')

   axe = fig.add_subplot(235, sharex=axe)
   axe.grid(True)
   log.plot('t', 'sig_dth_est*180/np.pi', axe, label=u'sigma dth_sim', color='green')
   log.plot('t', '-sig_dth_est*180/np.pi', axe, color='green')
   log.plot('t', '(dth_est-dth_sim)*180/np.pi', axe, label=u'delta dth_sim')
   axe.legend(loc='best')
   axe.set_title("Err. est. dth (deg/s)")
   axe.set_xlabel(u"Temps (s)")

   axe = fig.add_subplot(233)
   axe.grid(True)
   log.plot('t', 'biais_gyr_est*180/np.pi', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t', 't*0+%f' % (c.biais_gyr_deg), axe, label=u'simu')
   axe.set_title("Est. biais dth (deg/s)")
   axe.legend(loc='best')

   axe = fig.add_subplot(236, sharex=axe)
   axe.grid(True)
   log.plot('t', 'sig_biais_gyr_est*180/np.pi', axe, label=u'sigma biais', color='green')
   log.plot('t', '-sig_biais_gyr_est*180/np.pi', axe, color='green')
   log.plot('t', 'biais_gyr_est*180/np.pi-%f' % (c.biais_gyr_deg), axe, label=u'delta biais')
   axe.legend(loc='best')
   axe.set_title("Err. est. biais dth (deg/s)")
   axe.set_xlabel(u"Temps (s)")

   # fig.tight_layout()
   plt.show()


if __name__ == '__main__':
   main()
   # valide_modele_kal()

