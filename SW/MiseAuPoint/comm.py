#! /bin/env python2
# -*- coding: utf-8 -*-

import sys
import time

import numpy as np
import serial


ser = serial.Serial('/dev/cu.usbmodem1020251', 38400)
nb_val_max = 1000
time.sleep(1.)

t0 = time.time()
ser.write('START')
time.sleep(1.)
i = 0
while True:
   i += 1
   line = ser.readline()
   print i, line.strip()
   if 'STOP' in line:
      print u"Temps moyen d'une itération : %.3fms" % ((time.time()-t0)/nb_val_max*1000.)
      break
   
   