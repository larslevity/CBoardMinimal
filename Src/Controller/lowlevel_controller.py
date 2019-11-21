# -*- coding: utf-8 -*-
"""
Created on Tue Jul 23 13:39:59 2019

@author: AmP
"""
import numpy as np
import errno
import logging
import time
import threading

from Src.Hardware import sensors as sensors

from Src.Math import IMUcalc

from Src.Management import state_machine
from Src.Management.thread_communication import llc_ref
from Src.Management.thread_communication import llc_rec


from Src.Hardware.configuration import CHANNELset
from Src.Hardware.configuration import IMUset
from Src.Hardware.configuration import TSAMPLING


rootLogger = logging.getLogger()


def IMU_connection_test():
    imu_set = [imu for imu in IMUset]
    imu_used_ = [CHANNELset[name]['IMUs'] for name in CHANNELset]
    while [None] in imu_used_:
        imu_used_.remove([None])
    imu_used = list(np.unique([imu for subl in imu_used_ for imu in subl]))
    for imu in imu_used:
        if imu not in imu_set and imu is not None:
            raise KeyError(
                'IMU with name "{}"'.format(imu) + ' is used for angle' +
                'calculation, but is not in the set of connected IMUs')
    try:
        IMU = {}
        for name in IMUset:
            IMU[name] = sensors.MPU_9150(
                name=name, mplx_id=IMUset[name]['id'])
    except IOError:  # not connected
        IMU = False
    return IMU


class LowLevelController(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.sampling_time = TSAMPLING
        self.imu_in_use = None

    def is_imu_in_use(self):
        return self.imu_in_use

    def run(self):
        IMU = IMU_connection_test()
        self.imu_in_use = True if IMU else False

        def read_imu():
            for name in IMU:
                try:
                    llc_rec.acc[name] = IMU[name].get_acceleration()
                    llc_rec.gyr[name] = IMU[name].get_gyro()
                except IOError as e:
                    if e.errno == errno.EREMOTEIO:
                        rootLogger.exception(
                            'cant read imu device.' +
                            'Continue anyway ...Fail in [{}]'.format(name))
                    else:
                        rootLogger.exception('Sensor [{}]'.format(name))
                        rootLogger.error(e, exc_info=True)
                        raise e

        def angle_reference():
            rootLogger.info("Arriving in ANGLE_REFERENCE State. ")

            while llc_ref.state == 'ANGLE_REFERENCE':
                if IMU:
                    read_imu()

                    for name in IMUset:
                        ref = llc_ref.pressure[name]*90.
                        idx0, idx1 = CHANNELset[name]['IMUs']
                        rot_angle = CHANNELset[name]['IMUrot']
                        acc0 = llc_rec.acc[idx0]
                        acc1 = llc_rec.acc[idx1]
                        sys_out = IMUcalc.calc_angle(acc0, acc1, rot_angle)

                        llc_rec.u[name] = None
                        llc_rec.aIMU[name] = round(sys_out, 2)

                time.sleep(self.sampling_time)

            return llc_ref.state

        def feed_through():
            rootLogger.info("Arriving in FEED_THROUGH State: ")

            while llc_ref.state == 'FEED_THROUGH':
                # read
                if IMU:
                    read_imu()
                # write
                for name in CHANNELset:
                    pwm = llc_ref.pwm[name]
                    llc_rec.u[name] = pwm
                # meta
                time.sleep(self.sampling_time)

            return llc_ref.state

        def pause_state():
            """ do nothing. waiting for tasks """
            rootLogger.info("Arriving in PAUSE State: ")

            while llc_ref.state == 'PAUSE':
                if IMU:
                    read_imu()
                time.sleep(self.sampling_time)

            return llc_ref.state

        """ ---------------- ----- ------- ----------------------------- """
        """ ---------------- RUN STATE MACHINE ------------------------- """
        """ ---------------- ----- ------- ----------------------------- """

        automat = state_machine.StateMachine()
        automat.add_state('PAUSE', pause_state)
        automat.add_state('ANGLE_REFERENCE', angle_reference)
        automat.add_state('FEED_THROUGH', feed_through)
        automat.add_state('QUIT', None, end_state=True)
        automat.set_start('PAUSE')

        try:
            rootLogger.info('Run LowLevelCtr ...')
            automat.run()
        except Exception as e:
            rootLogger.exception(e)
            rootLogger.error(e, exc_info=True)
            raise
        rootLogger.info('LowLevelCtr is done ...')

    def kill(self):
        llc_ref.state = 'EXIT'
