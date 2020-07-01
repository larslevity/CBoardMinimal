# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 18:38:25 2019

@author: AmP
"""

import matplotlib.pyplot as plt
import numpy as np
import tikzplotlib

import load as my_load

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



fig, (ax1, ax2) = plt.subplots(nrows=2, sharex=True,
         gridspec_kw={'height_ratios': [3, 1]})

ax1.plot(time, alpha, color='blue')
ax2.plot(time, pref, color='red')

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

kwargs = {
    'strict': 1,
    'extra_tikzpicture_parameters': {},
    'extra_axis_parameters': {'height={3cm}', 'width={9cm}'},
    'extra_groupstyle_parameters': {'vertical sep={0pt}',
                                    'x descriptions at=edge bottom'}
        }

tikzplotlib.save('clb_B01_raw_data.tex', fig, standalone=True, **kwargs)

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
