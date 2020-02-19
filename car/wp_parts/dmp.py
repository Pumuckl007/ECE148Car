"""
dmp.py

Digital Motion Processor part for Donkeycar framework.
Uses the Invsense MPU 92/65 (9250 DMP, 6500 IMU).

@author: Saurabh Kulkarni
"""
# import python libraries
import time
import sys
from numpy import pi

# import rcpy library
# This automatically initizalizes the robotics cape
import rcmpupy as mpu9250

LOW_PASS_FILTER = 0.1
class MPU9265:
    def __init__(self):

        #donkeycar part specific
        self.on = True
        self.bearing = 0

        # defaults
        i2c_bus = 1
        gpio_interrupt_pin = 4
        #default enabled mag - for kiwi
        enable_magnetometer = True
        sample_rate = 100
        #TODO do we need fusion??
        enable_fusion = True
               
        # magnetometer ?
        mpu9250.initialize(i2c_bus = i2c_bus,
                           gpio_interrupt_pin = gpio_interrupt_pin,
                           enable_dmp = True,
                           dmp_sample_rate = sample_rate,
                           enable_fusion = enable_fusion,
                           enable_magnetometer = enable_magnetometer)

        #print("    Tait Bryan (rad) |", end='')
    
    def run(self):
        data = mpu9250.read()

        #formatted = '{0[0]:6.2f} {0[1]:6.2f} {0[2]:6.2f} |'
        #    .format(data['tb']), end='')
        #x,y,z = formatted.split()
        #x = float(x)
        #y = float(y)
        #self.bearing = z = float(z)
        self.bearing = data['tb'][2]
        #TODO is returning z bearing enough??? or do we need all 3?
        #if we calibrate this correctly, z should be enough. all 3 
        #would be tough to incorporate anyway
        #TODO verify that the entire stack is in radians not degrees
        return 360/2*pi *self.bearing


    def run_threaded(self):
        return self.bearing

    def update(self):
        while(self.on):
            data = mpu9250.read()
            #set LOW_PASS_FILTER TO 0 to disable
            self.bearing = LOW_PASS_FILTER*self.bearing + (1-LOW_PASS_FILTER)*data['tb'][2]

    def shutdown(self):
        self.on = False
