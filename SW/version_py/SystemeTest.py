import umatrix
import ulinalg

from libSystemControl.Systeme import ASysteme


# =============================================
# Définition du système
# =============================================
class SystemeTest (ASysteme):
   def __init__(self, lbda):
      ASysteme.__init__(self, [u'pos_sim'])
      
      self.A = -lbda
      self.K = lbda
      
   def comportement(self, x,u,t):
      th = x[0]
      
      dth = self.A*th + self.K*u[0,0]
      
      return umatrix.matrix([[dth]])
      
      
      