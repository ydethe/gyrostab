#! /bin/env python2
# -*- coding: utf-8 -*-

import collections

import numpy as np
from numpy import cos, sin, sqrt, dot
import scipy.linalg as lin

from SystemControl.System import ASystem


# =============================================
# Définition du système
# =============================================
class PenduleGyro (ASystem):
    __slots__ = []
    def __init__(self):
        ASystem.__init__(self, 'sys', name_of_outputs=['th_sim', 'dth_sim', 'ps_sim'])
        
        self.createParameter('masse_tot', 1.469)
        self.createParameter('omega', 10000.*2*np.pi/60)
        self.createParameter('OB',  16.6e-2)
        self.createParameter('OG', 14.56e-2)
        self.createParameter('J', 378.)
        self.createParameter('g0', 9.81)
        
        self.createParameter('rho', 2700.)
        self.createParameter('r1', 20.445e-3)
        self.createParameter('r2', 40.885e-3)
        self.createParameter('hauteur', 5.63e-2)
        # self.hauteur = sqrt(3*self.r1**2 + 3*self.r2**2)
        H2 = self.r2**2 + self.r1**2
        vol = self.hauteur*np.pi*(self.r2**2-self.r1**2)
        masse = vol*self.rho
        
        # Hypothèse 1 : h0**2 = 3*r1**2 + 3*r2**2
        # Hypothèse 2 : 2*J >> masse*(H2 + 2*self.OB**2)
        D = ((6*self.J-self.hauteur**2*masse)*H2-2*self.OB**2*self.hauteur**2*masse)/(6*H2)
        self.createParameter('A', self.masse_tot*self.OG*self.g0/D)
        self.createParameter('K', -masse*self.omega*H2/(D*2))
        
    def jacobian(self, t: float, x: np.array, u: np.array) -> np.array:
        T, dT, P = x
        u, = u
        return np.matrix([
           [0,	1,	0],
           [(12*self.masse_tot*self.OG*cos(T)*self.g0)/(((3*np.pi*sin(P)**2-6*np.pi)*self.hauteur*self.r2**4+(-np.pi*sin(P)**2*self.hauteur**3-12*np.pi*self.OB**2*self.hauteur)*self.r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self.hauteur*self.r1**4+(np.pi*sin(P)**2*self.hauteur**3+12*np.pi*self.OB**2*self.hauteur)*self.r1**2)*self.rho+12*self.J),	-((6*np.pi*cos(P)*self.hauteur*self.r2**4-2*np.pi*cos(P)*self.hauteur**3*self.r2**2-6*np.pi*cos(P)*self.hauteur*self.r1**4+2*np.pi*cos(P)*self.hauteur**3*self.r1**2)*self.rho*u)/(((3*np.pi*sin(P)**2-6*np.pi)*self.hauteur*self.r2**4+(-np.pi*sin(P)**2*self.hauteur**3-12*np.pi*self.OB**2*self.hauteur)*self.r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self.hauteur*self.r1**4+(np.pi*sin(P)**2*self.hauteur**3+12*np.pi*self.OB**2*self.hauteur)*self.r1**2)*self.rho+12*self.J),	((6*np.pi*cos(P)*sin(P)*self.hauteur*self.r2**4-2*np.pi*cos(P)*sin(P)*self.hauteur**3*self.r2**2-6*np.pi*cos(P)*sin(P)*self.hauteur*self.r1**4+2*np.pi*cos(P)*sin(P)*self.hauteur**3*self.r1**2)*self.rho*(((6*np.pi*self.hauteur*self.omega+6*np.pi*cos(P)*dT*self.hauteur)*self.r2**4-2*np.pi*cos(P)*dT*self.hauteur**3*self.r2**2+(-6*np.pi*self.hauteur*self.omega-6*np.pi*cos(P)*dT*self.hauteur)*self.r1**4+2*np.pi*cos(P)*dT*self.hauteur**3*self.r1**2)*self.rho*u-12*self.masse_tot*self.OG*sin(T)*self.g0))/(((3*np.pi*sin(P)**2-6*np.pi)*self.hauteur*self.r2**4+(-np.pi*sin(P)**2*self.hauteur**3-12*np.pi*self.OB**2*self.hauteur)*self.r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self.hauteur*self.r1**4+(np.pi*sin(P)**2*self.hauteur**3+12*np.pi*self.OB**2*self.hauteur)*self.r1**2)*self.rho+12*self.J)**2-((-6*np.pi*sin(P)*dT*self.hauteur*self.r2**4+2*np.pi*sin(P)*dT*self.hauteur**3*self.r2**2+6*np.pi*sin(P)*dT*self.hauteur*self.r1**4-2*np.pi*sin(P)*dT*self.hauteur**3*self.r1**2)*self.rho*u)/(((3*np.pi*sin(P)**2-6*np.pi)*self.hauteur*self.r2**4+(-np.pi*sin(P)**2*self.hauteur**3-12*np.pi*self.OB**2*self.hauteur)*self.r2**2+(6*np.pi-3*np.pi*sin(P)**2)*self.hauteur*self.r1**4+(np.pi*sin(P)**2*self.hauteur**3+12*np.pi*self.OB**2*self.hauteur)*self.r1**2)*self.rho+12*self.J)],
           [0,	0,	-(cos(P)*u)/sin(P)**2]]
        )
   
    def transition(self, t: float, x: np.array, u: np.array) -> np.array:
        th = x[0]
        dth1 = x[1]
        ps = x[2]
        
        # dps1 = np.clip(u[0,0]/np.sin(ps), -6*np.pi, 6*np.pi)
        # dps1 = 12*np.arctan(u[0,0]/np.sin(ps)/12)
        dps1 = u[0]/np.sin(ps)
        
        dth2 = (12*self.masse_tot*self.OG*self.g0*sin(th)+((-6*np.pi*dps1*dth1*self.hauteur*cos(ps)-6*np.pi*dps1*self.hauteur*self.omega)*sin(ps)*self.r2**4+2*np.pi*dps1*dth1*self.hauteur**3*cos(ps)*sin(ps)*self.r2**2+(6*np.pi*dps1*dth1*self.hauteur*cos(ps)+6*np.pi*dps1*self.hauteur*self.omega)*sin(ps)*self.r1**4-2*np.pi*dps1*dth1*self.hauteur**3*cos(ps)*sin(ps)*self.r1**2)*self.rho)/(((3*np.pi*self.hauteur*sin(ps)**2-6*np.pi*self.hauteur)*self.r2**4+(-np.pi*self.hauteur**3*sin(ps)**2-12*np.pi*self.OB**2*self.hauteur)*self.r2**2+(6*np.pi*self.hauteur-3*np.pi*self.hauteur*sin(ps)**2)*self.r1**4+(np.pi*self.hauteur**3*sin(ps)**2+12*np.pi*self.OB**2*self.hauteur)*self.r1**2)*self.rho+12*self.J)
        
        return np.array([dth1,dth2,dps1])
      
    def compute_output(self, t: float, state: np.array, inputs: dict) -> np.array:
        return state
        
        