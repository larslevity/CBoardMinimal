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

from Src.Management import state_machine
from Src.Management.thread_communication import llc_ref
from Src.Management.thread_communication import llc_rec


from Src.Hardware.configuration import CHANNELset
from Src.Hardware.configuration import IMUset
from Src.Hardware.configuration import TSAMPLING
from Src.Hardware.configuration import STARTSTATE

from csv_read_test import pattern_ref

from Src.Controller import ctrlib
from Src.Controller import calibration
from Src.Controller import compute_utils



rootLogger = logging.getLogger()

POTIS = {0: "P9_33"}
OUT = {0: "P8_13"}

for name in CHANNELset:
    PWM.start(OUT[name], 0, 25000)
    PWM.set_duty_cycle(OUT[name], 10.0)


def set_pressure_ref_via_poti():
    potis = {}
    for idx in POTIS:
        val = ADC.read(POTIS[idx])  # bug-> read twice
        val = round(ADC.read(POTIS[idx])*100)/100
        potis[idx] = val
    llc_ref.pressure = potis

def set_alpha_ref_via_poti():
    potis = {}
    for idx in POTIS:
        val = ADC.read(POTIS[idx])  # bug-> read twice
        val = round(ADC.read(POTIS[idx])*100)/100*120
        potis[idx] = val
    llc_ref.alpha = potis


def is_poti():
    for idx in POTIS:
        val = ADC.read(POTIS[idx])  # bug-> read twice
        val = round(ADC.read(POTIS[idx])*100)/100
    return True if val != 0 else False



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
        Ts = 0.03  # calc mean sampling time
        gamma = .07
        self.ell = 11.2
        self.gyrscale = 1/2500
        self.accscale = 9.81
        self.LP = [compute_utils.LP1n(Ts, gamma) for i in range(4)]  # lowpass filter
        self.Diff = compute_utils.Diff1(Ts)

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

        def calc_angle(self):                       #M "self" hinzugefügt
            if IMU:
                for name in CHANNELset:
                    idx0, idx1 = CHANNELset[name]['IMUs']
                    rot_angle = CHANNELset[name]['IMUrot']
                    acc0 = np.array(self.LP[0].filt(llc_rec.acc[idx0]))*self.accscale
                    acc1 = np.array(self.LP[1].filt(llc_rec.acc[idx1]))*self.accscale
                    gyr0 = np.array(self.LP[2].filt(llc_rec.gyr[idx0]))*self.gyrscale
                    gyr1 = np.array(self.LP[3].filt(llc_rec.gyr[idx1]))*self.gyrscale
                    domega_z = self.Diff.diff(gyr1[2]-gyr0[2])
                    last_alp = llc_rec.aIMU[name]

                    adynx, adyny = compute_utils.a_dyn(
                            domega_z, last_alp, self.ell)
                    acc1_static = [acc1[0]+adynx, acc1[1]+adyny, acc1[2]]
                    aIMU_filt = compute_utils.calc_angle(
                            acc0, acc1_static, rot_angle)

                    llc_rec.aIMU[name] = round(aIMU_filt, 2)

        read_imu()  # init recorder
        calc_angle(self)

        def PPIDBooster():
            rootLogger.info("Arriving in PPIDBooster State. ")
            booster = ctrlib.PressureBoost(version='big', tboost=.75)

            while llc_ref.state == 'PPIDBOOSTER':
                if IMU and is_poti():
                    read_imu()
                    calc_angle(self)
                    #referenz über pattern
                    pattern_ref(patternname='pattern_0.csv', alpha=True)
                    for name in CHANNELset:
                        aref = llc_ref.alpha[name]
                        pref = booster.get_reference(aref)
                        pwm = calibration.cut_off(int(100*pref), 100)
                        PWM.set_duty_cycle(OUT[name], pwm)
                        llc_rec.u[name] = pwm

                time.sleep(self.sampling_time)

            return llc_ref.state


        def PPID():
            rootLogger.info("Arriving in PPID State. ")

            while llc_ref.state == 'PPID':
                if IMU and is_poti():
                    read_imu()
                    calc_angle(self)
                    #referenz über pattern
                    pattern_ref(patternname='pattern_0.csv', alpha=True)
                    for name in CHANNELset:
                        aref = llc_ref.alpha[name]
                        pref = calibration.get_pressure(aref, version='big')
                        pwm = calibration.cut_off(int(100*pref), 100)
                        PWM.set_duty_cycle(OUT[name], pwm)
                        llc_rec.u[name] = pwm

                time.sleep(self.sampling_time)

            return llc_ref.state


        def CasPID():
            rootLogger.info("Arriving in CasPID State. ")
            PID = [0.008, 0.020, .01]
            CasCtr = ctrlib.PidController_WindUp(PID, TSAMPLING, max_output=1.)

            while llc_ref.state == 'CASPID':
                if IMU and is_poti():
                    read_imu()
                    calc_angle(self)
                    #referenz über pattern
                    pattern_ref(patternname='pattern_0.csv', alpha=True)
                    for name in CHANNELset:
                        aref = llc_ref.alpha[name]
                        pref = CasCtr.output(aref, llc_rec.aIMU[name])
                        pwm = pwm = calibration.cut_off(pref*100, 100)
                        PWM.set_duty_cycle(OUT[name], pwm)
                        llc_rec.u[name] = pwm

                time.sleep(self.sampling_time)

            return llc_ref.state


        def CasPIDClb():
            rootLogger.info("Arriving in CasPIDClb State. ")
            PID = [0.0204, 0.13, 0.0037]
            CasCtr = ctrlib.PidController_WindUp(PID, TSAMPLING, max_output=.4)

            while llc_ref.state == 'CASPIDCLB':
                if IMU:
                    read_imu()
                    calc_angle(self)
                    #referenz über pattern
#                    pattern_ref(patternname='pattern_0.csv', alpha=True)
                    #referenz über Poti
                    set_alpha_ref_via_poti()
                    for name in CHANNELset:
                        aref = llc_ref.alpha[name]
                        clb = calibration.get_pressure(aref, version='big')
                        pid = CasCtr.output(aref, llc_rec.aIMU[name])
                        pref = clb + pid
                        pwm = pwm = calibration.cut_off(pref*100, 100)
                        PWM.set_duty_cycle(OUT[name], pwm)
                        llc_rec.u[name] = pwm

                time.sleep(self.sampling_time)

            return llc_ref.state




        def POTIREF():
            rootLogger.info("Arriving in POTIREF State: ")

            while llc_ref.state == 'POTIREF':
                # read
                if IMU:
                    read_imu()
                    calc_angle(self)
                # referenz über Poti
                set_pressure_ref_via_poti()
                # write
                for name in CHANNELset:
                    pref = llc_ref.pressure[name]
                    PWM.set_duty_cycle(OUT[name], pref*100)
                    llc_rec.u[name] = pref*100
                time.sleep(self.sampling_time)

            return llc_ref.state

        def clb():
            rootLogger.info("Arriving in CLB State: ")

            while llc_ref.state == 'CLB':
                # read
                if IMU and is_poti():
                    read_imu()
                    calc_angle(self)
                pattern_ref(patternname='clb.csv', alpha=False)
                # write
                for name in CHANNELset:
                    pref = llc_ref.pressure[name]
                    PWM.set_duty_cycle(OUT[name], pref*100)
                    llc_rec.u[name] = pref*100
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
        automat.add_state('PPID', PPID)
        automat.add_state('POTIREF', POTIREF)
        automat.add_state('CASPID', CasPID)
        automat.add_state('CASPIDCLB', CasPIDClb)
        automat.add_state('PPIDBOOSTER', PPIDBooster)
        automat.add_state('CLB', clb)
        automat.add_state('EXIT', clean, end_state=True)
        
#        automat.set_start('FEED_THROUGH')
        automat.set_start(STARTSTATE)


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
