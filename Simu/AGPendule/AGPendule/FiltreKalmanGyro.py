import numpy as np
from numpy import sqrt
from control import ss,lqr

from libSystemControl.Estimator import KalmanFilter


__all__ = ['FiltreKalmanAngGyro', 'FiltreKalmanGyro']

# =============================================
# Définition de filtre de Kalman
# =============================================
class FiltreKalmanAngGyro (KalmanFilter):
   def __init__(self):
      KalmanFilter.__init__(self, ['th_est', 'dth_est', 'biais_ang_est', 'biais_gyr_est'])

   def setMatrices(self, A, K, dt):
      self._Ka = A
      self._Kk = K

      w = np.sqrt(A)*dt
      
      # A (n x n) matrice d'état
      Ma = np.matrix([[np.cosh(w),         np.sinh(w)/sqrt(A), 0., 0.],
                      [sqrt(A)*np.sinh(w), np.cosh(w),         0., 0.],
                      [0.,                 0.,                 1., 0.],
                      [0.,                 0.,                 0., 1.]])
      
      # B (n x m) matrice de updateController
      Mb = np.matrix(np.zeros((4,1)))
      Mb[0,0] = K/A*(np.cosh(w) - 1.)
      Mb[1,0] = K/sqrt(A)*np.sinh(w)
      
      # C (p x n) matrice d'observation
      Mc = np.matrix([[1., 0., 1., 0.],
                      [0., 1., 0., 1.]])
      
      # D (p x m) matrice d'action directe
      Md = np.matrix([[0.,0.]]).T
      
      # Q = np.diag([1.]*2)
      # R = np.diag([1.])
      # print("LQR")
      # K, S, E = lqr(Ma[0:2,0:2], Mb[0:2,:], Q, R)
      # print(K)
      
      # Q (n x n) covariance du bruit sur l'état
      Mq = np.matrix(np.eye(4))*1.e-4
      
      # R (p x p) covariance du bruit sur l'observation
      std_ang = 1.*np.pi/180.
      std_gyr = 1./10*np.pi/180.
      Mr = np.matrix(np.diag([std_ang,std_gyr])**2)

      KalmanFilter.setMatrices(self, Ma, Mb, Mc, Md, Mq, Mr, dt)

# =============================================
# Définition de filtre de Kalman
# =============================================
class FiltreKalmanGyro (KalmanFilter):
   def __init__(self):
      KalmanFilter.__init__(self, ['th_est', 'dth_est', 'biais_gyr_est'])

   def setMatrices(self, A, K, dt):
      self._Ka = A
      self._Kk = K

      w = np.sqrt(A)*dt
      
      # A (n x n) matrice d'état
      Ma = np.matrix([[np.cosh(w),         np.sinh(w)/sqrt(A), 0.],
                      [sqrt(A)*np.sinh(w), np.cosh(w),         0.],
                      [0.,                 0.,                 1.]])
      
      # B (n x m) matrice de updateController
      Mb = np.matrix(np.zeros((3,1)))
      Mb[0,0] = K/A*(np.cosh(w) - 1.)
      Mb[1,0] = K/sqrt(A)*np.sinh(w)
      
      # C (p x n) matrice d'observation
      Mc = np.matrix([[1., 0., 0.],
                      [0., 1., 1.]])
      
      # D (p x m) matrice d'action directe
      Md = np.matrix([[0.,0.]]).T
      
      # Q = np.diag([1.]*2)
      # R = np.diag([1.])
      # print("LQR")
      # K, S, E = lqr(Ma[0:2,0:2], Mb[0:2,:], Q, R)
      # print(K)
      
      # Q (n x n) covariance du bruit sur l'état
      Mq = np.matrix(np.eye(3))*1.e-4
      
      # R (p x p) covariance du bruit sur l'observation
      std_ang = 1.*np.pi/180.
      std_gyr = 1./10*np.pi/180.
      Mr = np.matrix(np.diag([std_ang,std_gyr])**2)

      KalmanFilter.setMatrices(self, Ma, Mb, Mc, Md, Mq, Mr, dt)


def test():
   import numpy as np
   import matplotlib.pyplot as plt

   from libSystemControl.Logger import Logger

   log = Logger()

   fs = 100.
   ns = 1000
   A = (2*np.pi)**2
   K = 1.
   w = np.sqrt(A)/fs
   std_ang = 5.
   std_gyr = 1./10
   biais_gyr = 1.

   # Init système
   th = 10.
   dth = 0.

   # Init Kalman
   kal = FiltreKalmanGyro()
   kal.setMatrices(A, K, 1./fs)
   kal.setEstimation(np.matrix([0.,0.,0.]).T, np.matrix(np.diag((1.,1.,1.)))/10., 0.)

   x_cons = 5.

   Xint = 0.
   for i in range(ns):
      # MAJ updateController
      Xest,t = kal.getEstimation()
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
      meas = np.matrix([th + np.random.normal()*std_ang, dth + biais_gyr + np.random.normal()*std_gyr]).T

      # MAJ Kalman
      kal.updateEstimator(meas, u, t)

      # MAJ Logger
      x,t = kal.getEstimation()
      th_est  = x[0,0]
      dth_est = x[1,0]

      Xint += (th_est-x_cons)/fs

      log.log('t', t)
      log.log('th_mes', meas[0,0])
      log.log('th', th)
      log.log('dth', dth)
      log.log('th_est', th_est)
      log.log('dth_est', dth_est)

   fig = plt.figure()

   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('t','th_mes', axe, label=u'mesuré', marker='+', linestyle='')
   log.plot('t','th_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('t','th', axe, label=u'simu')
   axe.legend()
   axe.set_xlabel(u"Temps (s)")

   plt.show()


if __name__ == '__main__':
   test()

