import umatrix


class ASysteme (object):
   def __init__(self, name_of_states):
      self._name_of_states = name_of_states
      self._u = 0.
      
      self._x = None
      self._t = None
      
   def get_num_states(self):
      return len(self._x)
      
   def comportement(self, x, u, t):
      u'''Loi de comportement des capteurs
      
      @param x \a array
         Vecteur d'etat du systeme
      
      @param u \a array
         Vecteur de commandes issu du controleur
      
      @param t (s) \a float
         Date courante
      
      @return dx/dt \a array
         Vecteur dx/dt
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour le systeme")
      
   def set_state(self, x0, t0):
      self._x = x0.copy()
      self._t = t0
      
   def integre_pas(self, u, dt):
      if self._x is None or self._t is None:
         raise SystemError(u"[ERREUR]Aucun etat initial defini pour le systeme")
         
      self._u = u.copy()
      
      self._x += dt*self.comportement(self._x, self._u, self._t)
      self._t += dt
      
   def get_state(self):
      return self._x.copy(), self._t
   
   def get_state_generator(self):
      X,t = self.get_state()
      for i in range(self.get_num_states()):
         yield self._name_of_states[i], X[i,0]
         
         
         