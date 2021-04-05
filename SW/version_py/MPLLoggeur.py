#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Loggeur.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np
import matplotlib.pyplot as plt


class Loggeur(object):
    @staticmethod
    def rad_to_deg(x):
        return x * 180.0 / np.pi

    def __init__(self):
        self.reset()

    def reset(self):
        self._data = {}

    def log(self, nom, val):
        if nom in self._data.keys():
            self._data[nom].append(val)
        else:
            self._data[nom] = [val]

    def getValue(self, nom, fct_conv=lambda x: x):
        expr_fct = "(lambda "
        expr_arg = "("
        for k in self._data.keys():
            expr_fct += """%s, """ % (k,)
            expr_arg += """np.array(self._data['%s']),""" % (k,)

        expr_fct = expr_fct[:-2] + ":" + nom + ")"
        expr_arg = expr_arg[:-1] + ")"
        val = eval(expr_fct + expr_arg)

        return val

    def plot(self, nom_x, nom_y, axe, **kwargs):
        val_x = self.getValue(nom_x)
        val_y = self.getValue(nom_y)

        axe.plot(val_x, val_y, **kwargs)
        axe.set_title(nom_y)
