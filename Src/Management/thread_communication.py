# -*- coding: utf-8 -*-
"""
Created on Thu Jul 25 10:46:32 2019

@author: AmP
"""

# from queue import Queue
from Src.Hardware.configuration import CHANNELset
from Src.Hardware.configuration import DiscreteCHANNELset
from Src.Hardware.configuration import IMUset
from Src.Hardware.configuration import STARTSTATE


class Borg:
    _shared_state = {}

    def __init__(self):
        self.__dict__ = self._shared_state


class LLCReference(Borg):
    def __init__(self):
        Borg.__init__(self)

        
        self.alpha = {name: 0. for name in CHANNELset}
        self.pwm = {name: 20. for name in CHANNELset}
        self.state = STARTSTATE

    def set_state(self, state):
        """ a state in ['PAUSE', 'ANGLE_REFERENCE', 'FEED_THROUGH', 'EXIT']
        """
        if state in ['PAUSE', 'ANGLE_REFERENCE',
                     'FEED_THROUGH', 'EXIT']:
            self.state = state
        else:
            raise KeyError('invalid state reference')


llc_ref = LLCReference()


class LLCRecorder(Borg):
    def __init__(self):
        Borg.__init__(self)

        self.u = {name: 0.0 for name in CHANNELset}
        self.aIMU = {name: None for name in CHANNELset}

        self.acc = {name: None for name in IMUset}
        self.gyr = {name: None for name in IMUset}


llc_rec = LLCRecorder()


class SystemConfig(Borg):
    def __init__(self):
        Borg.__init__(self)

        self.IMUsConnected = None
        self.LivePlotter = None
        self.ConsolePrinter = False

    def __str__(self):
        return (
"""
System Configuration as follows:
IMUs:\t\t {}connected
LivePlotter:\t {}connected
ConsolePrinter:\t {}abled
""".format(
    '' if self.IMUsConnected else 'not ', '' if self.LivePlotter else 'not ',
    'en' if self.ConsolePrinter else 'dis'))


sys_config = SystemConfig()
