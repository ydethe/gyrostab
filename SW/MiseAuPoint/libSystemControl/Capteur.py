#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Capteur.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np
from scipy import linalg as lin


class ACapteur (object):
   def __init__(self, name_of_mes, moy, cov):
      self._name_of_mes = name_of_mes
      self._moy = moy.copy()
      self._cov = cov.copy()
      if lin.norm(cov) == 0:
         self._cho = cov.copy()
      else:
         self._cho = np.matrix(lin.cholesky(cov))
   
   def get_num_mes(self):
      return len(self._moy)
      
   def comportement(self, x, t):
      u'''Loi de comportement des capteurs
      
      @param x \a array
         Vecteur d'etat du systeme
      
      @param t (s) \a float
         Date courante
      
      @return mes \a array
         Vecteur de mesures bruitee suivant la loi gaussienne donnee a l'instanciation
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour le capteur '%s'" % self._name)
      
   def mesure(self, x, t):
      z = self.comportement(x, t)
      
      bn = np.matrix(np.random.normal(size=len(z))).T
      bg = self._cho.T*bn + self._moy
      
      self._mes = z + bg
   
   def get_mesurement(self):
      return self._mes
      
   def get_mes_generator(self):
      X = self.get_mesurement()
      for i in range(self.get_num_mes()):
         yield self._name_of_mes[i], X[i,0]
      
      
   
