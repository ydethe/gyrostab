#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Systeme.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np
from scipy.integrate import odeint, ode


class ASysteme (object):
   def __init__(self, name_of_states):
      self._name_of_states = name_of_states
      self._u = 0.
      
      self._x = None
      self._t = None
      self._has_jacobian = hasattr(self, 'jacobien')
      
      if self._has_jacobian:
         self._integ = ode(lambda t, x, u:self.comportement(x,u,t), lambda t, x, u:self.jacobien(x,u,t))
      else:
         self._integ = ode(lambda t, x, u:self.comportement(x,u,t))
         
      self._integ.set_integrator('vode', nsteps=10000)
      # self._integ.set_integrator('dopri5', nsteps=10000)
      
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
      self._integ.set_initial_value(x0.copy(), t0)
      
   def integre_pas(self, u, dt):
      if self._x is None or self._t is None:
         raise SystemError(u"[ERREUR]Aucun etat initial defini pour le systeme")
         
      self._u = u.copy()
      
      self._integ.set_f_params(self._u).set_jac_params(self._u)
      if self._has_jacobian:
         self._integ.set_jac_params(self._u)
         
      self._x = self._integ.integrate(self._t+dt)
      self._t += dt
      
   def get_state(self):
      return self._x.copy(), self._t
   
   def get_state_generator(self):
      X,t = self.get_state()
      for i in range(self.get_num_states()):
         yield self._name_of_states[i], X[i,0]
      
      
      
if __name__ == '__main__':
   import doctest
   doctest.testmod()
   
   
