#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 13:01:48 2020

@author: ls
"""

import time
import matplotlib.pyplot as plt

from Src.Controller import ctrlib


Booster = ctrlib.PressureBoost(version='big', tboost=.5)

alpref = {0: 0 , 1:60} # , 2:0, 3:120, 4:120}

pref = []
t = []
t0 = time.time()

while time.time()-t0 < len(alpref):
    ti = time.time()-t0
    aref = alpref[int(ti)]
    pref.append(Booster.get_reference(aref))
    t.append(ti)
    time.sleep(.01)


plt.plot(t, pref)
plt.xlabel('time')
plt.ylabel('reference pressure')