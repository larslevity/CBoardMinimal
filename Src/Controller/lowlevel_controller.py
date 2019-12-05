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

try:
    import Adafruit_BBIO.PWM as PWM
    import Adafruit_BBIO.ADC as ADC
    ADC.setup()
except ImportError:
    pass

from Src.Hardware import MPU_9150 as sensors

from Src.Math import IMUcalc

from Src.Management import state_machine
from Src.Management.thread_communication import llc_ref
from Src.Management.thread_communication import llc_rec


from Src.Hardware.configuration import CHANNELset
from Src.Hardware.configuration import IMUset
from Src.Hardware.configuration import TSAMPLING

from csv_read_test import pattern_ref
from csv_read_test import generate_pose_ref
from csv_read_test import read_list_from_csv


rootLogger = logging.getLogger()

POTIS = {0: "P9_33"}
OUT = {0: "P8_13"}


for name in CHANNELset:
    PWM.start(OUT[name], 0, 25000)
    PWM.set_duty_cycle(OUT[name], 10.0)


def read_poti():
    potis = {}
    for idx in POTIS:
        val = ADC.read(POTIS[idx])  # bug-> read twice
        val = round(ADC.read(POTIS[idx])*100)/100
        potis[idx] = val
    llc_ref.ref = potis



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
            rootLogger.info("initialize IMU with mplx id: " +
                            str(IMUset[name]['id']))
            IMU[name] = sensors.MPU_9150(
                name=name, mplx_id=IMUset[name]['id'])
    except IOError:  # not connected
        rootLogger.info("failed")
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
                    if (e.errno == errno.EREMOTEIO 
                        or e.errno == errno.EWOULDBLOCK):
                        rootLogger.exception(
                            'cant read imu device.' +
                            'Continue anyway ...Fail in [{}]'.format(name))
                    else:
                        rootLogger.exception('Sensor [{}]'.format(name))
                        rootLogger.error(e, exc_info=True)
                        raise e

        def calc_angle():
            if IMU:
                for name in CHANNELset:
                    idx0, idx1 = CHANNELset[name]['IMUs']
                    rot_angle = CHANNELset[name]['IMUrot']
                    acc0 = llc_rec.acc[idx0]
                    acc1 = llc_rec.acc[idx1]
                    aIMU = IMUcalc.calc_angle(acc0, acc1, rot_angle)
                    llc_rec.aIMU[name] = round(aIMU, 2)

        read_imu()  # init recorder
        calc_angle()

        def angle_reference():
            rootLogger.info("Arriving in ANGLE_REFERENCE State. ")

            while llc_ref.state == 'ANGLE_REFERENCE':
                if IMU:
                    read_imu()
                    calc_angle()
                    for name in IMUset:
                        ref = llc_ref.pressure[name]*90.
                        sys_out = llc_rec.aIMU[name]
                        llc_rec.u[name] = None

                time.sleep(self.sampling_time)

            return llc_ref.state

        def feed_through():
            rootLogger.info("Arriving in FEED_THROUGH State: ")

            while llc_ref.state == 'FEED_THROUGH':
                # read
                if IMU:
                    read_imu()
                    calc_angle()
                read_poti()                    #referenz über Poti
#                pattern_ref()                   #referenz über pattern
                # write
                for name in CHANNELset:
                    ref = llc_ref.ref[name]
                    PWM.set_duty_cycle(OUT[name], ref*100)
                    llc_rec.u[name] = ref*100
                    
                # meta
                time.sleep(self.sampling_time)

            return llc_ref.state

        def pause_state():
            """ do nothing. waiting for tasks """
            rootLogger.info("Arriving in PAUSE State: ")

            while llc_ref.state == 'PAUSE':
                if IMU:
                    read_imu()
                    calc_angle()

                time.sleep(self.sampling_time)

            return llc_ref.state

        def clean():
            rootLogger.info('Clean PWM ...')
            PWM.cleanup()

        """ ---------------- ----- ------- ----------------------------- """
        """ ---------------- RUN STATE MACHINE ------------------------- """
        """ ---------------- ----- ------- ----------------------------- """

        automat = state_machine.StateMachine()
        automat.add_state('PAUSE', pause_state)
        automat.add_state('ANGLE_REFERENCE', angle_reference)
        automat.add_state('FEED_THROUGH', feed_through)
        automat.add_state('EXIT', clean, end_state=True)
        automat.set_start('FEED_THROUGH')

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
