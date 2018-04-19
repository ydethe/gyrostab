#! /bin/env python2
# -*- coding: utf-8 -*-
# $URL: svn://192.168.1.30/iProjets/trunk/Librairies/libSystemControl/src/libSystemControl/Loggeur.py $
# $Author: ydethe $
# $Date: 2018-03-05 08:21:26 +0100 (Lun, 05 mar 2018) $
# $Rev: 353 $
# $LastChangedRevision: 353 $

import numpy as np
import matplotlib.pyplot as plt


class Loggeur (object):
   def __init__(self):
      self.reset()
   
   def reset(self):
      self._data = {}
      
   def log(self, nom, t,val):
      if nom in self._data.keys():
         self._data[nom].append((t,val))
      else:
         self._data[nom] = [(t,val)]
      
   def plot(self, nom, axe, **kwargs):
      expr_fct = '(lambda '
      expr_arg = '('
      t = None
      for k in self._data.keys():
         if t is None:
            t = zip(*self._data[k])[0]
         expr_fct += '''%s, ''' % (k,)
         expr_arg += '''np.array(zip(*self._data['%s'])[1]),''' % (k,)
         
      expr_fct = expr_fct[:-2] + ':' + nom + ')'
      expr_arg = expr_arg[:-1] + ')'
      val = eval(expr_fct + expr_arg)
      
      axe.plot(t,val, **kwargs)
      axe.set_title(nom)
      
      
      
