#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Estimateur.py $
# $Author: ydethe $
# $Date: 2018-03-07 21:15:35 +0100 (Mer, 07 mar 2018) $
# $Rev: 355 $
# $LastChangedRevision: 355 $

import numpy as np
from numpy import cos, sin, cosh, sinh, sqrt, exp
from scipy import linalg as lin


class AEstimateur (object):
   def __init__(self, name):
      self._name = name
      
   def get_estimation(self):
      return self._xest.copy(), self._t
      
   def comportement(self, mes, u, t):
      u'''Loi de comportement des capteurs. Doit mettre à jour self._xest et self._t
      
      @param mes \a array
         Vecteur de mesures bruitee suivant la loi gaussienne donnee a l'instanciation
      
      @param u \a array
         Vecteur de commande issu du controleur
      
      @param t (s) \a float
         Date courante
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour l'estimateur '%s'" % self._name)
      
      

# =============================================
# Définition de filtre de Kalman
# =============================================
class FiltreKalman (AEstimateur):
   def __init__(self, name='kalman'):
      AEstimateur.__init__(self, name)
      
   def set_matrices(self, F, B, H, Q, R, dt):
      self._F = F
      self._B = B
      self._H = H
      self._Q = Q
      self._R = R
      
      self._dt = dt
      
   def set_state(self, x, P, t):
      self._n = len(x)
      self._xest = x.copy()
      
      self._P = P.copy()
      self._t = t
      
   def prediction(self, u):
      self._xest = self._F*self._xest + self._B*u
      self._P = self._F*self._P*self._F.T + self._Q
      
   def mise_a_jour(self, mes):
      if hasattr(mes, '__iter__'):
         z = mes.copy()
      else:
         z = np.matrix([[mes]])
      
      y = z - self._H*self._xest
      S = self._H*self._P*self._H.T + self._R
      K = self._P*self._H.T*S.I
      self._xest += K*y
      self._P = (np.matrix(np.eye(self._n)) - K*self._H)*self._P
      
   def comportement(self, mes, u, t):
      self._t = t + self._dt
      
      self.prediction(u)
      self.mise_a_jour(mes)
      
      
def test_cte():
   import matplotlib
   matplotlib.use('tkagg')

   import numpy as np
   import matplotlib.pyplot as plt
   
   from Loggeur import Loggeur

   log = Loggeur()
   
   fs = 100.
   ns = 1000
   
   # Init système
   cte = 1.
   
   # Init Kalman
   F = np.matrix([1.])
   B = np.matrix([0.])
   H = np.matrix([[1.]])
   Q = np.matrix(np.eye(1))*0
   R = np.matrix(np.eye(1))/100.
   
   kal = FiltreKalman()
   kal.set_matrices(F, B, H, Q, R, 1./fs)
   kal.set_state(np.matrix([0.]).T, np.matrix(np.diag((1.,)))/10., 0.)
   
   for i in range(ns):
      # MAJ commande
      Xest,t = kal.get_estimation()
      u = np.matrix([0.])
      
      # Mesure
      mes = np.matrix([cte + np.random.normal()/10.])
      
      # MAJ Kalman
      kal.comportement(mes, u, t)
      
      # MAJ Loggeur
      x,t = kal.get_estimation()
      th_est  = x[0,0]
      
      log.log('th_mes', t,mes[0,0])
      log.log('th', t,cte)
      log.log('th_est', t,th_est)
      
   fig = plt.figure()
   
   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('th_mes', axe, label=u'mesuré', marker='+', linestyle='')
   log.plot('th_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('th', axe, label=u'simu')
   axe.legend()
   axe.set_xlabel(u"Temps (s)")
   
   plt.show()
   
def test_ressort():
   import matplotlib
   matplotlib.use('tkagg')

   import numpy as np
   import matplotlib.pyplot as plt
   
   from Loggeur import Loggeur

   log = Loggeur()
   
   fs = 100.
   ns = 1000
   
   # Init système
   A = 2*np.pi
   K = 1.
   w = A/fs
   
   # Init Kalman
   F = np.matrix([[ cos(w)  , sin(w)/A],
                  [-A*sin(w), cos(w)  ]])
   B = np.matrix([K/A**2*(1. - cos(w)), K/A*sin(w)]).T
   H = np.matrix([[1., 0.]])
   Q = np.matrix(np.eye(2))*0
   
   # Init système
   th = 10.
   dth = 0.
   
   std_mes = 1./10
   R = np.matrix(np.eye(1))*std_mes**2
   
   kal = FiltreKalman()
   kal.set_matrices(F, B, H, Q, R, 1./fs)
   kal.set_state(np.matrix([0.,0.]).T, np.matrix(np.diag((1.,1.)))/10., 0.)
   
   x_cons = -5.
   
   Xint = 0
   for i in range(ns):
      # MAJ commande
      Xest,t = kal.get_estimation()
      a = A
      P = -(3*a**2-A**2)/K
      I = -a**3/K
      D = -(3*a)/K
      u = P*(Xest[0,:]-x_cons) + I*Xint + D*Xest[1,:]
      
      # MAJ système
      th2  =    cos(w)*th + sin(w)/A*dth + K/A**2*(1. - cos(w))*u[0,0]
      dth2 = -A*sin(w)*th +   cos(w)*dth + K/A*sin(w)*u[0,0]
      th = th2
      dth = dth2
      
      # Mesure
      mes = np.matrix([th + np.random.normal()*std_mes])
      
      # MAJ Kalman
      kal.comportement(mes, u, t)
      
      # MAJ Loggeur
      Xest,t = kal.get_estimation()
      th_est  = Xest[0,0]
      Xint += (th_est-x_cons)/fs
      
      log.log('th_mes', t,mes[0,0])
      log.log('th', t,th)
      log.log('th_est', t,th_est)
      
   fig = plt.figure()
   
   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('th_mes', axe, label=u'mesuré', marker='+', linestyle='')
   log.plot('th_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('th', axe, label=u'simu')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   plt.show()
   
def test_div():
   import matplotlib
   matplotlib.use('tkagg')

   import numpy as np
   import matplotlib.pyplot as plt
   
   from Loggeur import Loggeur

   log = Loggeur()
   
   fs = 100.
   ns = 1000
   
   # Init système
   A = 2*np.pi
   K = 1.
   w = A/fs
   
   # Init Kalman
   F = np.matrix([[cosh(w)  , sinh(w)/A],
                  [A*sinh(w), cosh(w)  ]])
   B = np.matrix([-K/A**2*(1. - cosh(w)), K/A*sinh(w)]).T
   H = np.matrix([[1., 0.]])
   Q = np.matrix(np.eye(2))*0
   
   # Init système
   th = 10.
   dth = 0.
   
   std_mes = 1./10
   R = np.matrix(np.eye(1))*std_mes**2
   
   kal = FiltreKalman()
   kal.set_matrices(F, B, H, Q, R, 1./fs)
   kal.set_state(np.matrix([0.,0.]).T, np.matrix(np.diag((1.,1.)))/10., 0.)

   x_cons = 5.
   
   Xint = 0.
   for i in range(ns):
      # MAJ commande
      Xest,t = kal.get_estimation()
      a = 5.
      P = (3*a**2+A**2)/K
      I = a**3/K
      D = 3*a/K
      u = -(P*(Xest[0,:]-x_cons) + I*Xint + D*Xest[1,:])
      
      # MAJ système
      th2  =   cosh(w)*th + sinh(w)/A*dth - K/A**2*(1. - cosh(w))*u[0,0]
      dth2 = A*sinh(w)*th +   cosh(w)*dth + K/A*sinh(w)*u[0,0]
      th = th2
      dth = dth2
      
      # Mesure
      mes = np.matrix([th + np.random.normal()*std_mes])
      
      # MAJ Kalman
      kal.comportement(mes, u, t)
      
      # MAJ Loggeur
      Xest,t = kal.get_estimation()
      th_est = Xest[0,0]
      
      Xint += (th_est-x_cons)/fs
      
      log.log('th_mes', t,mes[0,0])
      log.log('th', t,th)
      log.log('th_est', t,th_est)
      
   fig = plt.figure()
   
   axe = fig.add_subplot(111)
   axe.grid(True)
   log.plot('th_est', axe, label=u'estimé', marker='o', linestyle='')
   log.plot('th', axe, label=u'simu')
   log.plot('th_mes', axe, label=u'mesuré', marker='+', linestyle='')
   # log.plot('th_est-th', axe, label=u'simu')
   axe.legend(loc='best')
   axe.set_xlabel(u"Temps (s)")
   
   plt.show()
   

if __name__ == '__main__':
   # test_cte()
   # test_ressort()
   test_div()

