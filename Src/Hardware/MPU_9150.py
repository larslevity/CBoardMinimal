

try:
    import Adafruit_GPIO.I2C as Adafruit_I2C
except ImportError:
    print("Can't import Adafruit_I2C")

import subprocess
import time
import numpy as np

from Src.Hardware.configuration import IMUset
from Src.Hardware.configuration import CHANNELset



#GLOBAL VARIABLES
GRAVITIY_MS2 = 9.80665

#SCALE MODIFIERS
ACCEL_SCALE_MODIFIER_2G = 16384.0
ACCEL_SCALE_MODIFIER_4G = 8192.0
ACCEL_SCALE_MODIFIER_8G = 4096.0
ACCEL_SCALE_MODIFIER_16G = 2048.0

GYRO_SCALE_MODIFIER_250DEG = 131.0
GYRO_SCALE_MODIFIER_500DEG = 65.5
GYRO_SCALE_MODIFIER_1000DEG = 32.8
GYRO_SCALE_MODIFIER_2000DEG = 16.4

# Pre-defined ranges
ACCEL_RANGE_2G = 0x00
ACCEL_RANGE_4G = 0x08
ACCEL_RANGE_8G = 0x10
ACCEL_RANGE_16G = 0x18

GYRO_RANGE_250DEG = 0x00
GYRO_RANGE_500DEG = 0x08
GYRO_RANGE_1000DEG = 0x10
GYRO_RANGE_2000DEG = 0x18

#MPU-9150 REGISTERS
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
    
def i2cdetect():
    bashCommand = "i2cdetect -y -r 2"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    print(output)
    print(error)


class MultiPlexer(object):
    def __init__(self, address=0x70):
        self.i2c = Adafruit_I2C.get_i2c_device(address, busnum=2)

    def select(self, port_id):                                      ##note: port_id = multi ID
        self.i2c.write8(0, 1 << port_id)        


class MPU_9150(object):
    plexer = MultiPlexer()
    
    def __init__(self, name, mplx_id, address=0x68):
        power_mgmt_1 = 0x6B     # register power management of IMU
        self.name = name
        self.mplx_id = mplx_id    # plex ID of IMU
        # MultiPlexer schalten, um das Modul ansprechen zu koennen
        self.i2c = Adafruit_I2C.get_i2c_device(address, busnum=2)
        self.plexer.select(self.mplx_id)
        time.sleep(.1)
        # Power on of Acc
        self.i2c.write8(power_mgmt_1, 0x00)
        self.i2c.write8(ACCEL_CONFIG, ACCEL_RANGE_2G)               ##set accel sensitivity
        self.i2c.write8(GYRO_CONFIG, GYRO_RANGE_500DEG)             ##set gyro sensitivity

    def _read_word(self, reg):
        sens_bytes = self.i2c.readList(register=reg, length=2)
        msb = sens_bytes[0]
        lsb = sens_bytes[1]
        value = (msb << 8) + lsb
        return value

    def _read_word_2c(self, reg):
        val = self._read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def get_acceleration(self):
        self.plexer.select(self.mplx_id)

        acc_xout = self._read_word_2c(0x3b)/ACCEL_SCALE_MODIFIER_2G         ##convert raw data     
        acc_yout = self._read_word_2c(0x3d)/ACCEL_SCALE_MODIFIER_2G
        acc_zout = self._read_word_2c(0x3f)/ACCEL_SCALE_MODIFIER_2G
        return (acc_xout, acc_yout, acc_zout)

    def get_gyro(self):
        self.plexer.select(self.mplx_id)
        gyro_xout = self._read_word_2c(0x43)/GYRO_SCALE_MODIFIER_500DEG     ##convert raw data
        gyro_yout = self._read_word_2c(0x45)/GYRO_SCALE_MODIFIER_500DEG
        gyro_zout = self._read_word_2c(0x47)/GYRO_SCALE_MODIFIER_500DEG
        return (gyro_xout, gyro_yout, gyro_zout)



#if __name__ == "__main__":                      #array data
#    IMU = MPU_9150(0,1)
#    while True:
#        x_a, y_a, z_a = IMU.get_acceleration()
#        omega_dot_acc = arr.array('f', [x_a, y_a, z_a])
#        print(omega_dot_acc)
#        x_g, y_g, z_g = IMU.get_gyro()
#        omega_gyro = arr.array('f', [x_g, y_g, z_g])
#        print(omega_gyro)
#        time.sleep(.05)
        
    
if __name__ == "__main__":                    #single value data
    IMU = MPU_9150(0,1)
    while True:
        x_a, y_a, z_a = IMU.get_acceleration()
        s = 'x_acc: {}\n'.format(x_a)
        s = s + 'y_acc: {}\n'.format(y_a)
        s = s + 'z_acc: {}\n'.format(z_a)
        print(s)
        x_g, y_g, z_g = IMU.get_gyro()
        t = 'x_gyro: {}\n'.format(x_g)
        t = t + 'y_gyro: {}\n'.format(y_g)
        t = t + 'z_gyro: {}\n'.format(z_g)
        print(t)
        time.sleep(.05)
