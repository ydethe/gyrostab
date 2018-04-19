#! /bin/env python2
# -*- coding: utf-8 -*-

import collections

import numpy as np
from numpy import cos, sin, sqrt, dot
import scipy.linalg as lin

from libSystemControl.Systeme import ASysteme
from libSystemControl.Capteur import ACapteur
from libSystemControl.Controller import AController
from libSystemControl.Estimateur import FiltreKalman


# =============================================
# Définition du système
# =============================================
class PenduleGyro (ASysteme):
   def __init__(self):
      ASysteme.__init__(self, u'PenduleGyro')
      
      self._masse_tot = 1.
      self._hauteur = 0.07
      self._omega = 7500.*2*np.pi/60
      self._OB =  0.10
      self._OG = 0.05
      self._J = 0.
      self._g0 = 9.81
      
      r1 = 0.015
      r2 = sqrt(12*self._hauteur**2 - r1**2)
      vol = self._hauteur*np.pi*(r2**2-r1**2)
      # Masse volumique PLA : 1250kg/m^3
      masse = vol*1250.
      
      # mu[self._K] = h^(2-self._K)*integrale(r^self._K*rho(r)*dr)/self._K!, avec rho(r) la masse volumique
      mu1 = masse/(2*np.pi)
      self._mu1 = mu1
      # Test de la robustesse du modele à l'hypothèse d'égalité de mu1 et mu3
      self._mu3 = mu1
      
      self.A = 6*self._masse_tot*self._OG*self._g0/(np.pi*self._hauteur**2*(self._mu3) + 12*np.pi*self._OB**2*self._mu1-6*self._J)
      self.K = 2*np.pi*self._hauteur**2*self._mu1*self._omega/(np.pi*self._hauteur**2*(self._mu3) + 12*np.pi*self._OB**2*self._mu1-6*self._J)
      # print self.A, self.K
      
   def comportement(self, x,u,t):
      th = x[0]
      dth1 = x[1]
      ps = x[2]
      
      dpsi = u[0,0]/np.sin(ps)
      
      dth2 = ((2*np.pi*dpsi*dth1*self._hauteur**2*cos(ps)+2*np.pi*dpsi*self._hauteur**2*self._omega)*sin(ps)*self._mu3-2*np.pi*dpsi*dth1*self._hauteur**2*cos(ps)*sin(ps)*self._mu1+6*self._masse_tot*self._OG*self._g0*sin(th))/(np.pi*self._hauteur**2*((cos(ps)**2+1)*self._mu3+sin(ps)**2*self._mu1) + 12*np.pi*self._OB**2*self._mu1-6*self._J)
      
      return np.matrix([dth1,dth2,dpsi]).T
