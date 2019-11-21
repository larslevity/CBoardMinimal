# -*- coding: utf-8 -*-
"""
Created on Wed Jul 24 12:22:00 2019

@author: AmP
"""
import threading
import time

import errno
from socket import error as SocketError


from Src.GUI import datamanagement as mgmt

from Src.Management.thread_communication import llc_rec
from Src.Management.thread_communication import llc_ref


n_pc = len(llc_rec.p)           # proportional channels
n_imus = len(llc_rec.acc)       # IMUS connected


def prepare_data():
    r = [round(llc_ref.alpha[i], 2) for i in range(n_pc)]
    u = [round(llc_rec.u[i], 2) for i in range(n_pc)]
    acc = u = [round(llc_rec.acc[i], 2) for i in range(n_pc)]
    gyr = u = [round(llc_rec.gyr[i], 2) for i in range(n_pc)]

    rec_angle = llc_rec.aIMU
    aIMU = [round(rec_angle[i], 2) if rec_angle[i] else None
            for i in range(len(llc_rec.aIMU))]

    return (r, u, acc, gyr, aIMU)


class GUIPrinter(threading.Thread):
    def __init__(self, plotsock, IMU=False):
        threading.Thread.__init__(self)

        self.state = 'RUN'
        self.plotsock = plotsock
        self.IMU_connected = True if IMU else False

    def run(self):
        while self.state != 'EXIT':
            if self.plotsock:
                try:
                    sample = mgmt.rehash_record(*prepare_data(),
                                                IMU=self.IMU_connected)
                    _ = self.plotsock.send_sample(sample)
                except SocketError as err:
                    if err.errno != errno.ECONNRESET:
                        raise
                    print(err)
                    self.plotsock = None
                time.sleep(.1)

    def kill(self):
        self.state = 'EXIT'
        self.plotsock.close()
