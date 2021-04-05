import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from libSystemControl.Simulation import ASimulation

from AGPendule.FiltreKalmanGyro import FiltreKalmanGyro
from AGPendule.PenduleGyro import PenduleGyro
from AGPendule.Capteur import Capteur
from AGPendule.Controller import Controller


class Simulation (ASimulation):
   def comportement(self, x, t):
      return np.matrix([[0.*np.pi/180.]])

def cout(X, J0=None):
   if not J0 is None:
      sys.stdout.write(str(X))

   np.random.seed(1346)
   # Pas de temps
   dt = 0.1

   # Angle initial
   th0 = np.pi

   pend = PenduleGyro()
   pend.set_state(np.matrix([th0, 0., np.pi/2.]).T, 0.)

   kal = FiltreKalmanGyro()
   kal.set_state(np.matrix([th0,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., 0.)

   kal.set_matrices(pend.A, pend.K, dt)

   c = Capteur()

   ctrl = Controller(dt, pend, PID=X)

   sim = Simulation(dt, ctrl, pend, c, kal)

   sim.simule(20.)
   log = sim.get_loggeur()
   t = log.getValue('t')
   u = log.getValue('cmd')
   ps = log.getValue('ps_sim')
   dpsi = u/np.sin(ps)

   J = np.sum(ps**2) + np.sum(dpsi**2)
   if not J0 is None:
      J -= J0

   if not J0 is None:
      if J < 0:
         sys.stdout.write(' J0-%f\n' % abs(J))
      else:
         sys.stdout.write(' J0+%f\n' % J)
   return J


if __name__ == '__main__':
   X0 = np.array((10., 5., 5.))
   J0 = cout(X0)
   res = minimize(cout, X0, method='COBYLA', args=(J0,))
   print(res)
   cout(res.x, J0)



