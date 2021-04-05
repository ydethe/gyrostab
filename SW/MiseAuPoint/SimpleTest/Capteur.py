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
    def __init__(self):
        self.std_pos = 0.1

        moy = np.matrix([0.0]).T
        cov = np.matrix((np.diag([self.std_pos])) ** 2)
        ACapteur.__init__(self, [u"pos_mes"], moy, cov)

    def comportement(self, x, t):
        return x[0, :]
