import numpy as np

from libSystemControl.Sensors import ASensors


__all__ = ['Capteur']

# =============================================
# Définition des capteurs
# =============================================
class Capteur (ASensors):
   def __init__(self):
      self.std_ang_deg = 1.
      self.std_gyr_deg = 1./10
      self.biais_ang_deg = 0.
      self.biais_gyr_deg = -10.

      moy = np.matrix([self.biais_ang_deg, self.biais_gyr_deg]).T*np.pi/180.
      cov = np.matrix((np.diag([self.std_ang_deg,self.std_gyr_deg])*np.pi/180.)**2)
      ASensors.__init__(self, ['th_mes', 'dth_mes'], moy, cov)

   def behavior(self, x, u, t):
      return x[0:2,:]


def test():
   import numpy as np
   import matplotlib.pyplot as plt

   from libSystemControl.Logger import Logger

   log = Logger()

   fs = 100.
   ns = 100
   t = np.arange(ns)/fs

   cb = Capteur()

   for i in range(ns):
      x = 10.*np.pi/180.*np.sin(2*np.pi*t[i])
      dx = 2*np.pi*10.*np.pi/180.*np.cos(2*np.pi*t[i])
      cb.mesure(x=np.matrix([[x,dx]]).T,u=np.matrix([[0.]]),t=t[i])
      xmesb = cb.getMeasurements()

      log.log('t', t[i])
      log.log('x', x*180./np.pi)
      log.log('xmesb', xmesb[0,0]*180./np.pi)

   fig = plt.figure()

   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('t','xmesb', axe, label=u'mesuré + biais (=5)', marker='o', linestyle='')
   log.plot('t','x', axe, label=u'simu')
   axe.legend()
   axe.set_xlabel(u"Temps (s)")

   plt.show()


if __name__ == '__main__':
   test()

