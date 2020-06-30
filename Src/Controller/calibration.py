#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 11:54:33 2020

@author: ls
"""
import numpy as np


clb = {
    'big': [1.55177921e-10, -6.36649708e-08,  9.71552276e-06, -7.00987311e-04, 2.81959411e-02,  2.04603143e-01],
    }

def eval_poly(coef, x):
    if x < 0:
        return 0
    else:
        poly = np.poly1d(coef)
        return poly(x)

def cut_off(p, max_pressure):
    if p > max_pressure:
        # Warning('clb pressure > max_presse: I cutted it off')
        p_ = max_pressure
    elif p <= 0:
        p_ = 0.00
    else:
        p_ = round(p, 2)
    return p_


def get_pressure(alpha, version, max_pressure=1.1):
    try:
        if alpha == 0:
            return 0
        elif alpha is not None:
            p = eval_poly(clb[version], alpha)
            pressure = cut_off(p, max_pressure)
        return pressure
    except KeyError:
        raise NotImplementedError


def get_alpha(pressure, version):
    coeff = clb[version]
    poly = np.poly1d(coeff)
    roots = (poly - pressure).roots
    roots = roots[~np.iscomplex(roots)].real
    roots = roots[roots > 0]
    roots = roots[roots < 120]
    if len(roots) == 1:
        alp = (roots[0])
    elif len(roots) == 0:
        alp = 0
    else:
        alp = np.mean(roots)
    return alp


if __name__ == '__main__':
    from matplotlib import pyplot as plt

    version = 'big'

    for alpha in np.linspace(-30, 120, 50):
        p = get_pressure(alpha, version=version)
        alp_ = get_alpha(p, version)

        plt.figure(1)
        plt.plot(alpha, p*100, 'r.')
        plt.plot(alpha, alp_, 'k.')

    
    plt.figure(1)
    plt.xlabel('alpha [deg]')
    plt.ylabel('pressure(alpha) [bar*100] / alpha_ [deg]')
    plt.plot(0, 0, 'r.', label='p(alp) * 100')
    plt.plot(0, 0, 'k.', label='alp_(p(alp))')
    plt.legend()

    plt.show()
