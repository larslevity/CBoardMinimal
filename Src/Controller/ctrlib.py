#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 12:14:51 2020

@author: ls
"""

import abc
import numpy as np
import time

from Src.Controller import calibration


class Controller(object):
    """Base class for controllers. This defines the interface to controllers"""
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def reset_state(self):
        """
        Reset the states of the controller to zero
        """

    @abc.abstractmethod
    def set_maxoutput(self):
        """
        Set the maximal output of the Controller
        """

    @abc.abstractmethod
    def output(self, reference, system_output):
        return



class PressureBoost(object):
    def __init__(self, version, tboost=.5):
        self.coeff = [0.0131, 0.4583, 1.4503]
        self.version = version
        self.last_boost = time.time()
        self.steady_ref = 0
        self.last_steady_ref = 0
        self.ref = 0
        self.tboost = tboost
        self.is_active = False

    def boost_pressure(self, pbar):
        return sum([self.coeff[i]*pbar**i for i in range(len(self.coeff))])

    def get_reference(self, alpha):
        self.steady_ref = calibration.get_pressure(alpha, self.version)
        if self.steady_ref < self.last_steady_ref: # smaller
            self.ref = self.steady_ref
        elif self.steady_ref > self.last_steady_ref: # greater
            self.last_boost = time.time()
            self.ref = self.boost_pressure(self.steady_ref)
            self.is_active = True
        elif (time.time() - self.last_boost > self.tboost) and self.is_active:
            self.ref = self.steady_ref
            self.is_active = False

        self.last_steady_ref = self.steady_ref
        return self.ref
        


class PidController_WindUp(Controller):
    """
    A simple PID controller
    """
    def __init__(self, gain, tsampling, max_output):
        # Tuning Knobes
        self.Kp = gain[0]
        self.Ti = gain[1]
        self.Td = gain[2]
        self.max_output = max_output
        self.integral = 0.
        self.last_err = 0.
        self.last_out = 0.
        self.windup_guard = 0
        self.gam = .1   # pole for stability. Typically = .1
        self.tsampling = tsampling

    def set_maxoutput(self, maxoutput):
        self.max_output = maxoutput

    def reset_state(self):
        self.integral = 0.
        self.last_err = 0.
        self.windup_guard = 0.
        self.last_out = 0.

    def set_gain(self, gain):
        self.Kp = gain[0]
        self.Ti = gain[1]
        self.Td = gain[2]
        self.reset_state()

    def output(self, reference, system_output):
        # calc error
        err = reference - system_output
        # Derivative Anteil
        # diff = (err - self.last_err)/self.tsampling
        diff = (self.gam*self.Td - self.tsampling/2) / \
            (self.gam*self.Td + self.tsampling/2) * \
            self.last_out + \
            self.Td/(self.gam+self.tsampling/2)*(err-self.last_err)
        self.last_err = err
        # Integral Anteil
        integ = self.integral + self.tsampling / (2*self.Ti)*(err)
        self.integral = integ

        # Sum
        controller_output = self.Kp*(err + integ + diff)

        if np.abs(controller_output) > self.max_output:
            self.last_out = self.max_output*np.sign(controller_output)
        else:
            self.last_out = controller_output
        return self.last_out