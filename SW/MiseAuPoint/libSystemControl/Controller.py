#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Controller.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np


class AController (object):
   def __init__(self, name, dt, sys, capteurs, estimateurs):
      self._name = name
      self._dt = dt
      self._sys = sys
      self._capteurs = capteurs
      self._estimateurs = estimateurs
      
   def comportement(self, cons, Xest):
      u'''Loi de comportement des capteurs
      
      @param cons \a array
         Vecteur de consignes issu de l'utilisateur
      
      @param Xest \a array
         Etat estime
      
      @return u \a array
         Vecteur de commandes calculees
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour le controleur '%s'" % self._name)
      
   def integre_pas(self, th_c):
      x,t = self._sys.get_state()
      self._mes = self._capteurs.mesure(x, t)
      
      Xest,t = self._estimateurs.get_estimation()
      
      self._u = self.comportement(th_c, Xest)
      
      self._estimateurs.comportement(self._mes, self._u, t)
      
      self._sys.integre_pas(self._u, self._dt)
      
   def get_last_consigne(self):
      return self._u.copy()
   
   def get_last_measurement(self):
      return self._mes.copy()
      
