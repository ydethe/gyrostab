import numpy as np
from numpy import cos, sin

from libSystemControl.System import ASystem


__all__ = ['PenduleGyro']

# =============================================
# Définition du système
# =============================================
class PenduleGyro (ASystem):
   u'''Classe qui représente le pendule

   >>> sys = PenduleGyro()
   J = 0.005520
   h0² = 0.003m²
   3.r1² + 3.r2² = 0.006m²
   Rapport K/A = 1.915129
   f0 = 0.92Hz
   T0 = 1.09s
   >>> sys.setState(np.matrix([3., 0., np.pi/2.]).T, 0.)
   >>> dt = 0.01
   >>> ns = int(5./dt)
   >>> tps = np.empty(ns)
   >>> th = np.empty(ns)
   >>> for i in range(ns):
   ...    sys.updateSystem(np.matrix([[0.]]), dt)
   ...    x,t = sys.getState()
   ...    th[i] = x[0,0]
   ...    tps[i] = t
   >>> import matplotlib.pyplot as plt
   >>> _ = plt.plot(tps, th*180/np.pi)
   >>> plt.grid(True)
   >>> plt.show()

   '''
   def __init__(self, method='vode'):
      ASystem.__init__(self, ['th_sim', 'dth_sim', 'ps_sim'], method='vode')

      self._g0 = 9.81
      self._masse_tot = 1.469
      self._omega = 10000.*2*np.pi/60
      self._OB =  16.6e-2
      self._OG = 14.56e-2
      self._OB = 0.
      self._OG = 1.2e-2

      self._rho = 2700.
      self._r1 = 20.445e-3
      self._r2 = 40.885e-3
      self._hauteur = 5.63e-2
      # self._hauteur = sqrt(3*self._r1**2 + 3*self._r2**2)
      vol = self._hauteur*np.pi*(self._r2**2-self._r1**2)
      masse = vol*self._rho

      # J d'après CAO
      self._J = 5.52e-03
      # Estimation de J
      # self._J = self._OG**2*self._masse_tot
      print("J = %f" % self._J)
      # Vérification de l'hypothèse simplificatrice
      print("h0² = %.3fm²" % (self._hauteur**2))
      print("3.r1² + 3.r2² = %.3fm²" % (3*self._r1**2 + 3*self._r2**2))

      # Hypothèse : h0**2 = 3*r1**2 + 3*r2**2
      D = 12*self._J - 12*self._OB**2*masse - 2*masse*self._hauteur**2
      if D <= 1.e-8:
         raise ValueError(D)

      self.A = 12*self._masse_tot*self._OG*self._g0/D
      self.K = -2*masse*self._hauteur**2*self._omega/D
      print("Rapport K/A = %f" % (-self.K/self.A))
      f0 = np.sqrt(self.A)/(2*np.pi)
      print("f0 = %.2fHz" % f0)
      print("T0 = %.2fs" % (1/f0))

   def jacobien(self, x, u, t):
      T, dT, P = x
      return np.matrix([
         [0,	1,	0],
         [(12*self._masse_tot*self._OG*cos(T)*self._g0)/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J),	-((6*np.pi*cos(P)*self._hauteur*self._r2**4-2*np.pi*cos(P)*self._hauteur**3*self._r2**2-6*np.pi*cos(P)*self._hauteur*self._r1**4+2*np.pi*cos(P)*self._hauteur**3*self._r1**2)*self._rho*u)/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J),	((6*np.pi*cos(P)*sin(P)*self._hauteur*self._r2**4-2*np.pi*cos(P)*sin(P)*self._hauteur**3*self._r2**2-6*np.pi*cos(P)*sin(P)*self._hauteur*self._r1**4+2*np.pi*cos(P)*sin(P)*self._hauteur**3*self._r1**2)*self._rho*(((6*np.pi*self._hauteur*self._omega+6*np.pi*cos(P)*dT*self._hauteur)*self._r2**4-2*np.pi*cos(P)*dT*self._hauteur**3*self._r2**2+(-6*np.pi*self._hauteur*self._omega-6*np.pi*cos(P)*dT*self._hauteur)*self._r1**4+2*np.pi*cos(P)*dT*self._hauteur**3*self._r1**2)*self._rho*u-12*self._masse_tot*self._OG*sin(T)*self._g0))/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J)**2-((-6*np.pi*sin(P)*dT*self._hauteur*self._r2**4+2*np.pi*sin(P)*dT*self._hauteur**3*self._r2**2+6*np.pi*sin(P)*dT*self._hauteur*self._r1**4-2*np.pi*sin(P)*dT*self._hauteur**3*self._r1**2)*self._rho*u)/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J)],
         [0,	0,	-(cos(P)*u)/sin(P)**2]]
      )

   def behavior(self, x,u,t):
      th = x[0]
      dth1 = x[1]
      ps = x[2]

      # dps1 = u[0,0]/np.sin(ps)
      dps1 = np.clip(u[0,0]/np.sin(ps), -3, 3)

      dth2 = (12*self._masse_tot*self._OG*self._g0*sin(th)+((-6*np.pi*dps1*dth1*self._hauteur*cos(ps)-6*np.pi*dps1*self._hauteur*self._omega)*sin(ps)*self._r2**4+2*np.pi*dps1*dth1*self._hauteur**3*cos(ps)*sin(ps)*self._r2**2+(6*np.pi*dps1*dth1*self._hauteur*cos(ps)+6*np.pi*dps1*self._hauteur*self._omega)*sin(ps)*self._r1**4-2*np.pi*dps1*dth1*self._hauteur**3*cos(ps)*sin(ps)*self._r1**2)*self._rho)/(((3*np.pi*self._hauteur*sin(ps)**2-6*np.pi*self._hauteur)*self._r2**4+(-np.pi*self._hauteur**3*sin(ps)**2-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi*self._hauteur-3*np.pi*self._hauteur*sin(ps)**2)*self._r1**4+(np.pi*self._hauteur**3*sin(ps)**2+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J)

      return np.array([dth1,dth2,dps1])



if __name__ == '__main__':
   import doctest
   doctest.testmod()


