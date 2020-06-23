#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  3 11:20:11 2019

@author: bianca
"""

import csv
import time

from Src.Management.thread_communication import llc_ref

n_pc = len(llc_ref.ref) #pressure reference

class PatternMGMT(object):
    def __init__(self):
        self.last_process_time = time.time()
        self.process_time = 0
        self.initial_cycle = True
        self.pattern = None
        self.version = None
        self.idx = 0
        self.IMAGES = False
        self.init = False
        
mgmt = PatternMGMT()
        

def read_list_from_csv(filename):                                           #read csv in List
    out = []
    with open(filename) as f:
        reader = csv.reader(f, delimiter='\t')
        for row in reader:
            line = []
            for val in row:
                if val == 'True':
                    line.append(True)
                elif val == 'False':
                    line.append(False)
                else:
                    try:
                        line.append(float(val))
                    except ValueError:
                        line.append(val)
            out.append(line)
    return out


def generate_pose_ref(pattern, idx):                                        #generate pose ref (dic?) from list
    pos = pattern[idx]
    pv_task = {}

    local_min_process_time = pos[-1]
    ppos = pos[:n_pc]
    for kdx, pp in enumerate(ppos):
        pv_task[kdx] = pp
#        print(pv_task, local_min_process_time)                             #test print hier kommt die erste zeile des patterns an
    return pv_task, local_min_process_time

def pattern_ref(patternname):
    mgmt.pattern = read_list_from_csv(patternname)                               #festes Pattern zuweisen. kann ich das so machen?
    if mgmt.last_process_time + mgmt.process_time < time.time():
        if mgmt.initial_cycle:  # initial cycle                                    
            pattern = mgmt.pattern
            mgmt.idx = 0
            mgmt.initial_cycle = False
        else:  # normaler style
            pattern = mgmt.pattern
#            print(pattern)                                                      #test print -> no output -> das ganze läuft nur einmal durch
        # generate tasks
        pvtsk, processtime = generate_pose_ref(pattern, mgmt.idx)            #welche form hat pattern?
        # send to main thread
        llc_ref.ref = pvtsk                                                 #druckreferenz
        # organisation
        mgmt.process_time = processtime
        mgmt.last_process_time = time.time()
        mgmt.idx = mgmt.idx+1 if mgmt.idx < len(pattern)-1 else 0
        #print(mgmt.process_time, mgmt.last_process_time)                   #test print

    
