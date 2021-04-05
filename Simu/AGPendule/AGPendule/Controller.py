import numpy as np

from libSystemControl.Controller import AController


# =============================================
# Définition du controleur
# =============================================
class Controller (AController):
   def __init__(self, dt, sys, PID=None):
      AController.__init__(self, ['cmd'], dt)

      if PID is None:
         a = 7.
         self._P = -(3*a**2+sys.A)/sys.K
         self._I = -a**3/sys.K
         self._D = -3*a/sys.K
         print(u'Coefficients P, I, D : ', self._P,self._I,self._D)
      else:
         self._P,self._I,self._D = PID

      self._int_th = 0.

   def comportement(self,cons,Xest):
      th = Xest[0,0]
      dth1 = Xest[1,0]
      th_c = cons[0,0]

      u = self._P*(th-th_c) + self._I*self._int_th + self._D*dth1
      self._int_th += (th-th_c)*self._dt

      return np.matrix([[u]])

