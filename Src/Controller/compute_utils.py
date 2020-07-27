#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 11:47:51 2020

@author: ls
"""

import numpy as np
from scipy import signal

def calc_angle(vec1, vec2, rotate_angle=0., jump=np.pi*.5):
    theta = np.radians(rotate_angle)
    vec1 = rotate(vec1, theta)
    x1, y1, z1 = normalize(vec1)
    x2, y2, z2 = normalize(vec2)
    phi1 = np.arctan2(y1, x1)
    vec2 = rotate([x2, y2, 0], -phi1+jump)
    phi2 = np.degrees(np.arctan2(vec2[1], vec2[0]) - jump)

    alpha_IMU = -phi2

    return alpha_IMU


def normalize(vec):
    x, y, z = vec
    l = np.sqrt(x**2 + y**2 + z**2)
    return x/l, y/l, z/l


def rotate(vec, theta):
    c, s = np.cos(theta), np.sin(theta)
    return (c*vec[0]-s*vec[1], s*vec[0]+c*vec[1], vec[2])



class LP1(object):
    def __init__(self, Ts, gamma):
        self.a = (1-2*gamma/Ts)/(1+2*gamma/Ts)
        self.b = 1/(1+2*gamma/Ts)
        self.lastout = 0
        self.lastin = 0
    
    def filt(self, x):
        self.lastout = -self.a*self.lastout + self.b*(self.lastin + x)
        self.lastin = x
        return self.lastout


class LP1n(object):
    def __init__(self, Ts, gamma, n=3):
        ''' filter n-dim array '''
        self.LP1 = [LP1(Ts, gamma) for i in range(n)]

    def filt(self, x):
        filt = []
        for idx, val in enumerate(x):
            filt.append(self.LP1[idx].filt(val))
        return filt


class Diff1(object):
    def __init__(self, Ts):
        self.Ts = Ts
        self.lastout = 0
        self.lastin = 0

    def diff(self, x):
        self.lastout = 2/self.Ts*(x-self.lastin) - self.lastout
        self.lastin = x
        return self.lastout


def diff_array(x, Ts):
    d1 = Diff1(Ts)
    dx = []
    for val in x:
        dx.append(d1.diff(val))
    del d1
    return np.array(dx)


def lowpass_array(x, Ts=0.03, gamma=.1):
    lp1 = LP1(Ts, gamma)
    x_filt = []
    for val in x:
        x_filt.append(lp1.filt(val))
    del lp1
    return np.array(x_filt)



def JAnorm(alp):
    return 2/alp**2*(1-2*(np.sin(alp/2)*np.cos(alp/2))/alp)


def a_dyn(omega, alp, l):
    alp = np.deg2rad(alp)
    if abs(alp) < 0.01:
        absa = 1/3*omega*l
    else:
        absa = JAnorm(alp)*omega*l*alp/(2*np.sin(alp/2))
    return np.array([-np.sin(alp/2), np.cos(alp/2)])*absa


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    gamma = .1
    Ts = .03
    fs = 1/Ts*2*np.pi
    wn = 2/gamma/fs
    Kp = 1
    
    myb = [1, 1]
    mya = [1+2*gamma/Ts, 1-2*gamma/Ts]
    
    anaw, anah = signal.freqs([1], [gamma, 1])
    
    
    
    b, a = signal.butter(1, wn, 'low')
    w, h = signal.freqz(b, a, fs=fs)
    myw, myh = signal.freqz(myb, mya, fs=fs)
    plt.semilogx(w, 20 * np.log10(abs(h)), '--', label='butter')
    plt.semilogx(myw, 20 * np.log10(abs(myh)), label='my')
    plt.semilogx(anaw, 20 * np.log10(abs(anah)), ':', label='my')
    plt.title('Butterworth filter frequency response')
    plt.xlabel('Frequency [radians / second]')
    plt.ylabel('Amplitude [dB]')
    plt.margins(0, 0.1)
    plt.grid(which='both', axis='both')
    plt.axvline(1/gamma, color='green') # cutoff frequency
    plt.show()
