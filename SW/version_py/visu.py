#! /bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

from MPLLoggeur import Loggeur


log = Loggeur()

f = open('log.txt', 'r')
lines = f.readlines()
f.close()

for i in range(len(lines)):
   nom,sval = lines[i].strip().split(',')
   log.log(nom, float(sval))
   
fig = plt.figure()
axe = fig.add_subplot(211)
axe.grid(True)

log.plot('t', '180./np.pi*pos_est', axe, label=u'Estimation', marker='o', linestyle='')
log.plot('t', '180./np.pi*pos_sim', axe, label=u'Simulation')
log.plot('t', '180./np.pi*pos_mes', axe, label=u'Mesure', marker='+', linestyle='')
axe.legend(loc='best')

axe = fig.add_subplot(212, sharex=axe)
axe.grid(True)

log.plot('t', '180./np.pi*biais_pos_est', axe, label=u'Biais pos')
log.plot('t', '180./np.pi*biais_vel_est', axe, label=u'Biais vel')
axe.legend(loc='best')

plt.show()
