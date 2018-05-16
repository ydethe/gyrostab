import umatrix
import ulinalg
from math import cos, sin, cosh, sinh, sqrt, exp


class AEstimateur (object):
   def __init__(self, name_of_states):
      self._name_of_states = name_of_states
      
   def get_num_states(self):
      return len(self._xest)
      
   def set_state(self, x, P, t):
      raise SystemError(u"[ERREUR]Methode 'set_state' non implementee pour l'estimateur")
      
   def get_estimation(self):
      return self._xest.copy(), self._t
   
   def get_estimation_generator(self):
      X,t = self.get_estimation()
      for i in range(self.get_num_states()):
         yield self._name_of_states[i], X[i,0]
      
   def comportement(self, mes, u, t):
      u'''Loi de comportement des capteurs. Doit mettre à jour self._xest et self._t
      
      @param mes \a array
         Vecteur de mesures bruitee suivant la loi gaussienne donnee a l'instanciation
      
      @param u \a array
         Vecteur de commande issu du controleur
      
      @param t (s) \a float
         Date courante
         
      '''
      raise SystemError(u"[ERREUR]Methode 'comportement' non implementee pour l'estimateur")
      

# =============================================
# Définition de filtre de Kalman
# =============================================
class FiltreKalman (AEstimateur):
   def __init__(self, name_of_states):
      AEstimateur.__init__(self, name_of_states)
      
   def set_matrices(self, A,B,C,D, Q, R, dt):
      self._A = A
      self._B = B
      self._C = C
      self._D = D
      self._Q = Q
      self._R = R
      
      self._dt = dt
      
   def set_state(self, x, P, t):
      self._n = len(x)
      self._xest = x.copy()
      
      self._P = P.copy()
      self._t = t
      
   def prediction(self, u):
      self._xest = self._A*self._xest + self._B*u
      self._P = self._A*self._P*self._A.T + self._Q
      
   def mise_a_jour(self, mes, u):
      y = mes - (self._C*self._xest + self._D*u)
      S = self._C*self._P*self._C.T + self._R
      dS, SI = ulinalg.det_inv(S)
      K = self._P*self._C.T*SI
      self._xest += K*y
      self._P = (ulinalg.eye(self._n) - K*self._C)*self._P
   
   def getCovariance(self):
      return self._P.copy()
   
   def getEcartsTypes(self, num_var=None):
      if num_var is None:
         return umatrix.matrix([[sqrt(self._P[i,i]) for i in range(self._n)]])
      else:
         return sqrt(self._P[num_var,num_var])
      
   def comportement(self, mes, u, t):
      self._t = t + self._dt
      
      self.prediction(u)
      self.mise_a_jour(mes, u)
      
      
      