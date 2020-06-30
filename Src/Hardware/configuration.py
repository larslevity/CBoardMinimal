# -*- coding: utf-8 -*-
"""
Created on Thu Jul 25 12:29:50 2019

@author: AmP
"""


## CHANNELS


MAX_PRESSURE = 1.   # bar
MAX_CTROUT = 0.50     # [10V]
TSAMPLING = 0.001     # [sec]
PID = [0.0117, 1.012, 0.31]

#STARTSTATE = 'PPID'
#STARTSTATE = 'CLB'
STARTSTATE = 'CASPIDCLB'


'''
Positions of IMUs:
<       ^       >
0 ----- 1 ----- 2
        |
        |
        |
3 ------4 ------5
<       v       >
In IMUcalc.calc_angle(acc0, acc1, rot_angle), "acc0" is turned by rot_angle
'''

IMUset = {
    0: {'id': 0},
    1: {'id': 1},
    }
CHANNELset = {
    0: {'pin': 'P9_22', 'ctr': PID, 'IMUs': [0, 1], 'IMUrot': 0},
    }
