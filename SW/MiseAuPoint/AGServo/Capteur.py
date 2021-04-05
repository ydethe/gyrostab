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
# Définition des capteurs
# =============================================
class Capteur(ACapteur):
    def __init__(self, lbda):
        self.std_pos = 0.00448355705097
        self.std_vel = 0.0014239412603
        self.lbda = lbda

        moy = np.matrix([0.0, 4.0 * np.pi / 180.0]).T
        cov = np.matrix((np.diag([self.std_pos, self.std_vel])) ** 2)
        ACapteur.__init__(self, [u"pos_mes", "vel_mes"], moy, cov)

    def comportement(self, x, u, t):
        th = x[0, 0]
        dth = -self.lbda * (th - u[0, 0])
        return np.matrix([th, dth]).T
