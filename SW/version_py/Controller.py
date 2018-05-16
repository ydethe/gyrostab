import umatrix
import ulinalg
from math import cos, sin, sqrt

from libSystemControl.Controller import AController


# =============================================
# Définition du controleur
# =============================================
class Controller (AController):
   def __init__(self, dt, sys):
      AController.__init__(self, ['ctrl'], dt)
      
      a = 2.
      self._P = (sys.A-2*a)/sys.K
      self._I = -a**2/sys.K
      
      self._int_th = 0.
      
   def comportement(self,cons,Xest):
      th = Xest[0,0]
      th_c = cons[0,0]
      
      u = self._P*(th-th_c) + self._I*self._int_th
      self._int_th += (th-th_c)*self._dt
      
      return umatrix.matrix([[u]])

