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


n_pc = len(llc_rec.u)           # proportional channels
n_imus = len(llc_rec.acc)       # IMUS connected


def prepare_data():
    aref = [round(llc_ref.alpha[i], 2) for i in range(n_pc)]
    pref = [round(llc_ref.pressure[i], 2) for i in range(n_pc)]
    u = [round(llc_rec.u[i], 2) for i in range(n_pc)]

    accx = [round(llc_rec.acc[i][0], 2) for i in range(n_imus)]
    accy = [round(llc_rec.acc[i][1], 2) for i in range(n_imus)]
    accz = [round(llc_rec.acc[i][2], 2) for i in range(n_imus)]
    gyrx = [round(llc_rec.gyr[i][0], 2) for i in range(n_imus)]
    gyry = [round(llc_rec.gyr[i][1], 2) for i in range(n_imus)]
    gyrz = [round(llc_rec.gyr[i][2], 2) for i in range(n_imus)]

    aIMU = [round(llc_rec.aIMU[i], 2) if llc_rec.aIMU[i] else None
            for i in range(len(llc_rec.aIMU))]

    # record = rehash_record(r, u, aIMU, accx, accy, accz, gyrx, gyry, gyrz)
    return (pref, aref, u, aIMU, accx, accy, accz, gyrx, gyry, gyrz)


class GUIPrinter(threading.Thread):
    def __init__(self, plotsock):
        threading.Thread.__init__(self)

        self.state = 'RUN'
        self.plotsock = plotsock

    def run(self):
        while self.state != 'EXIT':
            if self.plotsock:
                try:
                    sample = mgmt.rehash_record(*prepare_data())
                    _ = self.plotsock.send_sample(sample)
                except SocketError as err:
                    if err.errno != errno.ECONNRESET:
                        raise
                    print(err)
                    self.plotsock = None
                time.sleep(mgmt.GUISampling)

    def kill(self):
        self.state = 'EXIT'
        self.plotsock.close()
