"""
module for data management
"""

import time
from collections import deque


GUISampling = .005


def merge_multiple_dicts(dicts):
    super_dict = {}
    for d in dicts:
        for key, value in iter(d.items()):
            super_dict[key] = value
    return super_dict


def rehash_record(pref=[None]*1, aref=[None]*1, motor_in=[None]*1,
                  alphaIMU=[None]*1,
                  accx=[None]*2, accy=[None]*2, accz=[None]*2,
                  gyrx=[None]*2, gyry=[None]*2, gyrz=[None]*2):
    # record = rehash_record(r, u, aIMU, accx, accy, accz, gyrx, gyry, gyz)

    pref = {'pref{}'.format(idx): px for idx, px in enumerate(pref)}
    aref = {'aref{}'.format(idx): px for idx, px in enumerate(aref)}
    u = {'u{}'.format(idx): px for idx, px in enumerate(motor_in)}
    t = {'time': time.time()}

    record = merge_multiple_dicts([pref, aref, u, t])
    aIMU = {'aIMU{}'.format(idx): px for idx, px in enumerate(alphaIMU)}
    acx = {'accx{}'.format(idx): px for idx, px in enumerate(accx)}
    acy = {'accy{}'.format(idx): px for idx, px in enumerate(accy)}
    acz = {'accz{}'.format(idx): px for idx, px in enumerate(accz)}
    gyx = {'gyrx{}'.format(idx): px for idx, px in enumerate(gyrx)}
    gyy = {'gyry{}'.format(idx): px for idx, px in enumerate(gyry)}
    gyz = {'gyrz{}'.format(idx): px for idx, px in enumerate(gyrz)}
    record = merge_multiple_dicts(
            [record, aIMU, acx, acy, acz, gyx, gyy, gyz])

    return record


class GUIRecorder(object):
    """
    GUIData provide a customized set of data structure for GUI
    control of the GeckoBot system.
    The GUIData is only for the GUI.
    """
    def __init__(self):
        """
        """
        self.recorded = {}
        self.max_idx = 0
        self.StartStop = False
        self.StartStopIdx = [None, None]
        self.record = False
        self.record_file = None

    def append(self, sample, maxlen=1000):
        """
        Append a sample to recorder.

        Args:
            sample (dict): Dictionary of all recorded values in a sample
        """
        in_rec = True
        for key in sample:
            if key not in self.recorded:
                self.recorded[key] = {'val': deque([], maxlen), 'len': 0}
                in_rec = False
            self.recorded[key]['val'].append(sample[key])
            self.recorded[key]['len'] += 1
            if self.recorded[key]['len'] > self.max_idx:
                val_len = self.recorded[key]['len']
                self.max_idx = val_len if val_len < maxlen else maxlen
        return in_rec
