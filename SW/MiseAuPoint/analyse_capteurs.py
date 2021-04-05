#! /bin/env python2
# -*- coding: utf-8 -*-

import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


def lit_donnees(fic):
    f = open(fic, "r")
    lines = f.readlines()
    f.close()

    n = len(lines)
    ang = np.empty(n, dtype=np.float32)
    vang = np.empty(n, dtype=np.float32)

    for i in range(n):
        ang[i], vang[i] = [float(x) for x in lines[i].strip().split()]

    return ang, vang


def main():
    ang, vang = lit_donnees("donnees_capteurs.txt")
    print np.mean(ang), np.std(ang) * np.pi / 180.0
    print np.mean(vang), np.std(vang) * np.pi / 180.0


if __name__ == "__main__":
    main()
