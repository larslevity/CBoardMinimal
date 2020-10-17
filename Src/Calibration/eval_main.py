# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 18:38:25 2019

@author: AmP
"""

import matplotlib.pyplot as plt
import numpy as np
import tikzplotlib

import load as my_load


class LP1(object):
    def __init__(self, Ts, gamma=.07):
        self.a = (1-2*gamma/Ts)/(1+2*gamma/Ts)
        self.b = 1/(1+2*gamma/Ts)
        self.lastout = 0
        self.lastin = 0
    
    def filt(self, x):
        self.lastout = -self.a*self.lastout + self.b*(self.lastin + x)
        self.lastin = x
        return self.lastout


# %% LOAD
filename = 'clb_B01.csv'
data = my_load.read_csv(filename)

alpha = np.array(data['aIMU0']) - data['aIMU0'][0]  # remove offset
pref =  np.array(data['u0'])/100.
time = np.array(data['time']) - data['time'][0]


# %% Remove all 0 refs

alpha = alpha[pref!=0]
time = time[pref!=0]
pref = pref[pref!=0]


# %% Find index of changing pref
idx = np.array([1 if pref[i] != pref[i-1] else 0 for i in range(1, len(pref))])
idx = np.insert(idx, 0, 0)
idx[-1] = 1

plt.figure(1)
plt.plot(idx)
plt.plot(pref)


# %% Take n previous measurements before change of pref
n = 50
jdx = np.zeros(np.shape(idx))
alpha_mean = []
pref_mean = []
time_mean = []
alpha_filt = []

Ts = time[-1]/len(time)
lp = LP1(Ts)

for alp in alpha:
    alpha_filt.append(lp.filt(alp))

for i, val in enumerate(idx):
    if val == 1:
        jdx[i-n:i] = np.ones((n))
        alpha_mean.append(np.mean(alpha[i-n:i]))
        pref_mean.append(np.mean(pref[i-n:i]))
        time_mean.append(np.mean(time[i-n:i]))

plt.plot(jdx)
plt.xlabel('index')
plt.ylabel('pref')


# %% Validate

plt.figure(2)
plt.plot(alpha, pref)
plt.plot(alpha_mean, pref_mean)
plt.xlabel('pref')
plt.ylabel('alpha')
plt.grid()




# %%

fig, ((ax1, ax3), (ax2, ax4)) = plt.subplots(nrows=2, ncols=2, sharex='col',
         gridspec_kw={'height_ratios': [3, 1]})

#ax1.plot(time, alpha, color='blue')
ax1.plot(time, alpha_filt, color='blue')
ax2.plot(time, pref, color='red')

#ax1.plot(time_mean, alpha_mean, 'o', label='used measurements', color='orange', alpha=.4)
#ax2.plot(time_mean, pref_mean, 'o', label='used measurements', color='orange', alpha=.4)


ax2.grid()
ax2.set_ylabel(r'$\bar{p}$ (bar)', color='red')
ax2.tick_params('y', colors='red')
ax2.set_yticks([0, .5, 1])
ax2.set_ylim(-.1, 1.1)
ax2.set_xlim(0, 180)
ax2.set_xlabel('time (s)')
ax2.set_xticks([0, 60, 120, 180])

ax1.grid()
ax1.set_ylabel(r'$\alpha$ ($^\circ$)', color='blue')
ax1.tick_params('y', colors='blue')
ax1.set_yticks([0, 50, 100, 150])
ax1.set_ylim(-15, 175)
ax1.set_xlim(0, 180)
#ax1.set_xticks([])


## zoom
start, end = 0, -1
ax3.plot(time[start:end], alpha_filt[start:end], color='blue')
ax4.plot(time[start:end], pref[start:end], color='red')

ax3.plot(time_mean, alpha_mean, 'o', label='used measurements', color='orange', alpha=.7)
ax4.plot(time_mean, pref_mean, 'o', label='used measurements', color='orange', alpha=.7)

ax3.set_xlim(160, 174)
ax3.set_ylim(105, 140)
ax4.set_ylim(.9, 1.01)
ax3.set_yticks([110, 120, 130])

ax4.grid()
ax4.tick_params('y', colors='red')
ax4.set_xlabel('time (s)')

ax3.grid()
ax3.tick_params('y', colors='blue')


kwargs = {
    'strict': 1,
    'extra_tikzpicture_parameters': {},
    'extra_axis_parameters': {'height={5cm}', 'width={5cm}'},
    'extra_groupstyle_parameters': {'vertical sep={5pt}',
                                    'x descriptions at=edge bottom'}
        }

tikzplotlib.save('clb_B01_raw_data_mod.tex', fig, standalone=True, **kwargs)

# %% calc coeffs
deg = 5

coef = np.polyfit(alpha_mean, pref_mean, deg)
clb = list(coef)

coef_s = ['%1.3e' % c for c in coef]
print('Actuator %s:\t%s' % (filename, coef_s))
poly = np.poly1d(coef)
alp = np.linspace(-20, 180, 100)

plt.figure('CLB'+str(filename))
plt.plot(alpha, pref, ':', label='measurements', color='blue')
plt.plot(alpha_mean, pref_mean, 'o', label='used measurements', color='orange')
plt.plot(alp, poly(alp), '-', label='polynomial fit', color='green')

plt.grid()
plt.xlim((-20, 180))
plt.xticks([0, 50, 100, 150])
plt.yticks([0, .5, 1])
plt.ylim((-.1, 1.3))
plt.xlabel(r'bending angle $\alpha$ ($^\circ$)')
plt.ylabel(r'$\bar{p}$ (bar)')
plt.legend(loc='lower right')

kwargs = {
    'strict': 1,
    'extra_tikzpicture_parameters': {},
    'extra_axis_parameters': {'height={7cm}', 'width={9cm}'},
        }

tikzplotlib.save('clb_B01_eval.tex', standalone=True, **kwargs)


#
#plt.show()
#
#print(clb)
