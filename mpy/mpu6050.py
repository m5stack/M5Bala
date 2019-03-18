#
# This file is part of MicroPython MPU9250 driver
# Copyright (c) 2018 Mika Tuupola
# Copyright (c) 2018 0x1abin (added the yaw,picth,roll api and complementary filtering)
#
# Licensed under the MIT license:
#   http://www.opensource.org/licenses/mit-license.php
#
# Project home:
#   https://github.com/tuupola/micropython-mpu9250
#

"""
MicroPython I2C driver for MPU6500 6-axis motion tracking device
"""

__version__ = "0.2.0"

# pylint: disable=import-error
import ustruct
from machine import I2C, Pin
from micropython import const
import math
import utime as time
# pylint: enable=import-error

_SMPLRT_DIV = const(0x19)
_CONFIG = const(0x1a)
_PWR_MGMT_1 = const(0x6b)
_GYRO_CONFIG = const(0x1b)
_ACCEL_CONFIG = const(0x1c)
_ACCEL_CONFIG2 = const(0x1d)
_INT_PIN_CFG = const(0x37)
_ACCEL_XOUT_H = const(0x3b)
_ACCEL_XOUT_L = const(0x3c)
_ACCEL_YOUT_H = const(0x3d)
_ACCEL_YOUT_L = const(0x3e)
_ACCEL_ZOUT_H = const(0x3f)
_ACCEL_ZOUT_L = const(0x40)
_TEMP_OUT_H = const(0x41)
_TEMP_OUT_L = const(0x42)
_GYRO_XOUT_H = const(0x43)
_GYRO_XOUT_L = const(0x44)
_GYRO_YOUT_H = const(0x45)
_GYRO_YOUT_L = const(0x46)
_GYRO_ZOUT_H = const(0x47)
_GYRO_ZOUT_L = const(0x48)
_WHO_AM_I = const(0x75)

#_ACCEL_FS_MASK = const(0b00011000)
ACCEL_FS_SEL_2G = const(0b00000000)
ACCEL_FS_SEL_4G = const(0b00001000)
ACCEL_FS_SEL_8G = const(0b00010000)
ACCEL_FS_SEL_16G = const(0b00011000)

_ACCEL_SO_2G = 16384  # 1 / 16384 ie. 0.061 mg / digit
_ACCEL_SO_4G = 8192  # 1 / 8192 ie. 0.122 mg / digit
_ACCEL_SO_8G = 4096  # 1 / 4096 ie. 0.244 mg / digit
_ACCEL_SO_16G = 2048  # 1 / 2048 ie. 0.488 mg / digit

#_GYRO_FS_MASK = const(0b00011000)
GYRO_FS_SEL_250DPS = const(0b00000000)
GYRO_FS_SEL_500DPS = const(0b00001000)
GYRO_FS_SEL_1000DPS = const(0b00010000)
GYRO_FS_SEL_2000DPS = const(0b00011000)

_GYRO_SO_250DPS = 131
_GYRO_SO_500DPS = 65.5
_GYRO_SO_1000DPS = 32.8
_GYRO_SO_2000DPS = 16.4

# Used for enablind and disabling the i2c bypass access
_I2C_BYPASS_MASK = const(0b00000010)
_I2C_BYPASS_EN = const(0b00000010)
_I2C_BYPASS_DIS = const(0b00000000)

SF_G = 1
SF_M_S2 = 9.80665  # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 57.295779513082  # 1 rad/s is 57.295779513082 deg/s


class MPU6050:
    """Class which provides interface to MPU6500 6-axis motion tracking device."""

    def __init__(
        self, i2c=None, address=0x68,
        accel_fs=ACCEL_FS_SEL_2G, gyro_fs=GYRO_FS_SEL_500DPS,
        accel_sf=SF_G, gyro_sf=SF_DEG_S
    ):
        if i2c:
            self.i2c = i2c
        else:
            self.i2c = I2C(sda=21, scl=22, speed=400000)
        self.address = address

        # if 0x71 != self.whoami:
        #     raise RuntimeError("MPU6500 not found in I2C bus.")

        # Init
        self._register_char(_SMPLRT_DIV, 0x00)
        self._register_char(_CONFIG, 0x00)
        self._accel_so = self._accel_fs(accel_fs)
        self._gyro_so = self._gyro_fs(gyro_fs)
        self._accel_sf = accel_sf
        self._gyro_sf = gyro_sf
        self._register_char(_PWR_MGMT_1, 0x01)

        # Enable I2C bypass to access for MPU9250 magnetometer access.
        # char = self._register_char(_INT_PIN_CFG)
        # char &= ~_I2C_BYPASS_MASK # clear I2C bits
        # char |= _I2C_BYPASS_EN
        # self._register_char(_INT_PIN_CFG, char)
        self.preInterval = time.ticks_us()
        self.accCoef = 0.02
        self.gyroCoef = 0.98
        self.angleGyroX = 0
        self.angleGyroY = 0
        self.angleGyroZ = 0
        self.angleX = 0
        self.angleZ = 0
        self.angleY = 0
        self.gyroXoffset = 0
        self.gyroYoffset = 0
        self.gyroZoffset = 0

    def setGyroOffsets(self, x, y, z):
        self.gyroXoffset = x
        self.gyroYoffset = y
        self.gyroZoffset = z

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
        return values in g if constructor was provided `accel_sf=SF_M_S2`
        parameter.
        """
        so = self._accel_so
        sf = self._accel_sf

        xyz = self._register_three_shorts(_ACCEL_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    @property
    def gyro(self):
        """
        X, Y, Z radians per second as floats.
        """
        so = self._gyro_so
        sf = self._gyro_sf

        xyz = self._register_three_shorts(_GYRO_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    @property
    def ypr(self):
        """
        yaw, pitch, roll as floats.
        """
        accX, accY, accZ = self.acceleration

        angleAccX = math.atan2(accY, accZ + abs(accX)) * SF_RAD_S
        angleAccY = math.atan2(accX, accZ + abs(accY)) * (-SF_RAD_S)

        gyroX, gyroY, gyroZ = self.gyro
        gyroX -= self.gyroXoffset
        gyroY -= self.gyroYoffset
        gyroZ -= self.gyroZoffset

        interval = (time.ticks_us() - self.preInterval) / 1000000
        self.preInterval = time.ticks_us()

        self.angleGyroX += gyroX * interval
        self.angleGyroY += gyroY * interval
        self.angleGyroZ += gyroZ * interval

        self.angleX = (self.gyroCoef * (self.angleX + gyroX * interval)) + (self.accCoef * angleAccX)
        self.angleY = (self.gyroCoef * (self.angleY + gyroY * interval)) + (self.accCoef * angleAccY)
        self.angleZ = self.angleGyroZ

        return tuple([self.angleZ, self.angleX, self.angleY])

    @property
    def whoami(self):
        """ Value of the whoami register. """
        return self._register_char(_WHO_AM_I)

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack(">h", buf)[0]

        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">hhh", buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return buf[0]

        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _accel_fs(self, value):
        self._register_char(_ACCEL_CONFIG, value)

        # Return the sensitivity divider
        if ACCEL_FS_SEL_2G == value:
            return _ACCEL_SO_2G
        elif ACCEL_FS_SEL_4G == value:
            return _ACCEL_SO_4G
        elif ACCEL_FS_SEL_8G == value:
            return _ACCEL_SO_8G
        elif ACCEL_FS_SEL_16G == value:
            return _ACCEL_SO_16G

    def _gyro_fs(self, value):
        self._register_char(_GYRO_CONFIG, value)

        # Return the sensitivity divider
        if GYRO_FS_SEL_250DPS == value:
            return _GYRO_SO_250DPS
        elif GYRO_FS_SEL_500DPS == value:
            return _GYRO_SO_500DPS
        elif GYRO_FS_SEL_1000DPS == value:
            return _GYRO_SO_1000DPS
        elif GYRO_FS_SEL_2000DPS == value:
            return _GYRO_SO_2000DPS

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
