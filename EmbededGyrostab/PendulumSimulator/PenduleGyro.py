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
      ASysteme.__init__(self, ['th_sim', 'dth_sim', 'ps_sim'])
      
      self._masse_tot = 1.469
      self._omega = 10000.*2*np.pi/60
      self._OB =  16.6e-2
      self._OG = 14.56e-2
      self._J = 378.
      self._g0 = 9.81
      
      self._rho = 2700.
      self._r1 = 20.445e-3
      self._r2 = 40.885e-3
      self._hauteur = 5.63e-2
      # self._hauteur = sqrt(3*self._r1**2 + 3*self._r2**2)
      H2 = self._r2**2 + self._r1**2
      vol = self._hauteur*np.pi*(self._r2**2-self._r1**2)
      masse = vol*self._rho
      
      # Hypothèse 1 : h0**2 = 3*r1**2 + 3*r2**2
      # Hypothèse 2 : 2*J >> masse*(H2 + 2*self._OB**2)
      D = ((6*self._J-self._hauteur**2*masse)*H2-2*self._OB**2*self._hauteur**2*masse)/(6*H2)
      self.A = self._masse_tot*self._OG*self._g0/D
      self.K = -masse*self._omega*H2/(D*2)
      
   def jacobien(self, x, u, t):
      T, dT, P = x
      return np.matrix([
         [0,	1,	0],
         [(12*self._masse_tot*self._OG*cos(T)*self._g0)/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J),	-((6*np.pi*cos(P)*self._hauteur*self._r2**4-2*np.pi*cos(P)*self._hauteur**3*self._r2**2-6*np.pi*cos(P)*self._hauteur*self._r1**4+2*np.pi*cos(P)*self._hauteur**3*self._r1**2)*self._rho*u)/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J),	((6*np.pi*cos(P)*sin(P)*self._hauteur*self._r2**4-2*np.pi*cos(P)*sin(P)*self._hauteur**3*self._r2**2-6*np.pi*cos(P)*sin(P)*self._hauteur*self._r1**4+2*np.pi*cos(P)*sin(P)*self._hauteur**3*self._r1**2)*self._rho*(((6*np.pi*self._hauteur*self._omega+6*np.pi*cos(P)*dT*self._hauteur)*self._r2**4-2*np.pi*cos(P)*dT*self._hauteur**3*self._r2**2+(-6*np.pi*self._hauteur*self._omega-6*np.pi*cos(P)*dT*self._hauteur)*self._r1**4+2*np.pi*cos(P)*dT*self._hauteur**3*self._r1**2)*self._rho*u-12*self._masse_tot*self._OG*sin(T)*self._g0))/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J)**2-((-6*np.pi*sin(P)*dT*self._hauteur*self._r2**4+2*np.pi*sin(P)*dT*self._hauteur**3*self._r2**2+6*np.pi*sin(P)*dT*self._hauteur*self._r1**4-2*np.pi*sin(P)*dT*self._hauteur**3*self._r1**2)*self._rho*u)/(((3*np.pi*sin(P)**2-6*np.pi)*self._hauteur*self._r2**4+(-np.pi*sin(P)**2*self._hauteur**3-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self._hauteur*self._r1**4+(np.pi*sin(P)**2*self._hauteur**3+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J)],
         [0,	0,	-(cos(P)*u)/sin(P)**2]]
      )
   
   def comportement(self, x,u,t):
      th = x[0]
      dth1 = x[1]
      ps = x[2]
      
      # dps1 = np.clip(u[0,0]/np.sin(ps), -6*np.pi, 6*np.pi)
      # dps1 = 12*np.arctan(u[0,0]/np.sin(ps)/12)
      dps1 = u[0,0]/np.sin(ps)
      
      dth2 = (12*self._masse_tot*self._OG*self._g0*sin(th)+((-6*np.pi*dps1*dth1*self._hauteur*cos(ps)-6*np.pi*dps1*self._hauteur*self._omega)*sin(ps)*self._r2**4+2*np.pi*dps1*dth1*self._hauteur**3*cos(ps)*sin(ps)*self._r2**2+(6*np.pi*dps1*dth1*self._hauteur*cos(ps)+6*np.pi*dps1*self._hauteur*self._omega)*sin(ps)*self._r1**4-2*np.pi*dps1*dth1*self._hauteur**3*cos(ps)*sin(ps)*self._r1**2)*self._rho)/(((3*np.pi*self._hauteur*sin(ps)**2-6*np.pi*self._hauteur)*self._r2**4+(-np.pi*self._hauteur**3*sin(ps)**2-12*np.pi*self._OB**2*self._hauteur)*self._r2**2+(6*np.pi*self._hauteur-3*np.pi*self._hauteur*sin(ps)**2)*self._r1**4+(np.pi*self._hauteur**3*sin(ps)**2+12*np.pi*self._OB**2*self._hauteur)*self._r1**2)*self._rho+12*self._J)
      
      return np.array([dth1,dth2,dps1])
      
      
      