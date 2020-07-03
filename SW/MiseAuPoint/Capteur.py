#! /bin/env python2
# -*- coding: utf-8 -*-

import collections

import numpy as np
from numpy import cos, sin, sqrt, dot
import scipy.linalg as lin

from SystemControl.Sensors import ASensors


# =============================================
# Définition des capteurs
# =============================================
class Capteur (ASensors):
    def __init__(self):
        self.std_ang_deg = 1.
        self.std_gyr_deg = 1./10
        self.biais_gyr_deg = -5
        
        moy = np.array([0., self.biais_gyr_deg*np.pi/180.])
        cov = (np.diag([self.std_ang_deg,self.std_gyr_deg])*np.pi/180.)**2
        ASensors.__init__(self, 'cpt', name_of_outputs=['th_mes', 'dth_mes'])
        
        self.mean = moy
        self.cov = cov
      
    def compute_state(self, t1: float, t2: float, inputs: dict) -> np.array:
        state = self.getDataForInput(inputs, 'state')
        return state[0:2]

