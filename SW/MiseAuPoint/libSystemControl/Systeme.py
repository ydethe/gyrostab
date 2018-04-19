#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Systeme.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np
from scipy.integrate import odeint


class ASysteme (object):
   def __init__(self, name):
      self._name = name
      self._u = 0.
      
      self._x = None
      self._t = None
      
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
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour le systeme '%s'" % self._name)
      
   def set_state(self, x0, t0):
      self._x = x0.copy()
      self._t = t0
      
   def integre_pas(self, u, dt):
      if self._x is None or self._t is None:
         raise SystemError(u"[ERREUR]Aucun etat initial defini pour '%s'" % self._name)
         
      self._u = u.copy()
      x = odeint(lambda x, t:np.array(self.comportement(x,self._u,t).flat), np.array(self._x.flat), [self._t, self._t+dt])
      self._x = np.matrix(x[1,:]).T
      self._t += dt
      
   def get_state(self):
      return self._x.copy(), self._t
      
      
      
if __name__ == '__main__':
   import doctest
   doctest.testmod()
   
   
