#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Controller.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np

from Loggeur import Loggeur


class ASimulation (object):
   def __init__(self, dt, ctrl, sys, capteurs, estimateurs):
      self._dt = dt
      self._controller = ctrl
      self._sys = sys
      self._capteurs = capteurs
      self._estimateurs = estimateurs
      self._log = Loggeur()
      
   def comportement(self, x, t):
      u'''Loi de comportement des capteurs
      
      @param x \a array
         Estimateur du systeme
      
      @param t (s) \a float
         Date courante
      
      @return cons \a array
         Vecteur de consigne applique au systeme
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour la simulation")
      
   def simule(self, tfin):
      # Nombre de pas de temps simulés
      ns = int(tfin/self._dt)
      
      for i in range(ns):
         t_sim = i*self._dt
         
         # Lecture de l'état physique du système
         # (inconnu dans la réalité)
         x_sys,t_sys = self._sys.get_state()
         
         # Simulation d'une mesure
         self._capteurs.mesure(x_sys, t_sys)
         mes = self._capteurs.get_mesurement()
         
         # Mise à jour de l'estimateur avec la dernière consigne appliquée
         u = self._controller.get_last_consigne()
         self._estimateurs.comportement(mes, u, t_sys)
         x_est,t_est = self._estimateurs.get_estimation()
         
         for name,cmde in self._controller.get_last_consigne_generator():
            self._log.log(name, cmde)
         
         for name,state in self._estimateurs.get_estimation_generator():
            self._log.log(name, state)
         
         for name,state in self._sys.get_state_generator():
            self._log.log(name, state)
         
         for name,mes in self._capteurs.get_mes_generator():
            self._log.log(name, mes)
         
         self._log.log('t', t_sim)
         
         # Calcul de la nouvelle consigne
         th_c = self.comportement(x_est, t_est)
         
         # Calcul de la nouvelle commande
         self._controller.commande(th_c, x_est)
         u = self._controller.get_last_consigne()
         
         # Simulation du système d'un pas de temps plus tard
         self._sys.integre_pas(u, self._dt)
         
         # self._log.log('sig_th', kal.getEcartsTypes(0))
         
   def get_loggeur(self):
      return self._log
      
      
      