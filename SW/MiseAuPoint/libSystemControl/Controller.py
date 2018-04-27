#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Controller.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np


class AController (object):
   def __init__(self, _name_of_cmd, dt):
      self._name_of_cmd = _name_of_cmd
      self._dt = dt
      self._u = np.matrix([[0.]*len(self._name_of_cmd)])
   
   def get_num_cmd(self):
      return len(self._name_of_cmd)
      
   def commande(self, cons, Xest):
      self._u = self.comportement(cons, Xest)
      
   def comportement(self, cons, Xest):
      u'''Loi de comportement des capteurs
      
      @param cons \a array
         Vecteur de consignes issu de l'utilisateur
      
      @param Xest \a array
         Etat estime
      
      @return u \a array
         Vecteur de commandes calculees
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour le controleur")
      
   def get_last_consigne(self):
      return self._u.copy()
      
   def get_last_consigne_generator(self):
      u = self.get_last_consigne()
      for i in range(self.get_num_cmd()):
         yield self._name_of_cmd[i], u[i,0]
      
      
   