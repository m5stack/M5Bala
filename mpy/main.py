from micropython import const
import os
import time
import machine
import _thread
from m5bala import M5Bala

# Init the I2C bus
i2c = machine.I2C(sda=machine.Pin(21), scl=machine.Pin(22), freq=400000)

# Balance START
m5bala = M5Bala(i2c)
m5bala.start(thread=True)
