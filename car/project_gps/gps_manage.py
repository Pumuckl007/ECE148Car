#!/usr/bin/env python3
"""
gps_manage.py

Script to control donkey car with GPS navigation. Waypoints are set with GPS coordinates in degrees.

Call: gps_manage.py -drive
"""

# import GPS Planner and other DK parts
import donkeycar as dk
from gps_parts.gps import GPS
from gps_parts.planner import Planner
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

# other important modules
import serial
import pynmea2
import time
import threading


def drive(cfg, goalLocation):
    """
    drive(cfg, goalLocation)

    Add GPS, Planner, and actuator parts and call DK Vehicle.py to run car.
    @param: cfg - configuration file from dk calibration
            goalLocation - list of GPS coordinates in degrees
    @return: None
    """
    # initialize vehicle
    V = Vehicle()

    # GPS is a DK part that will poll GPS data from serial port
    # and output current location in radians.
    gps = GPS(cfg.BAUD_RATE, cfg.PORT, cfg.TIMEOUT)

    # Planner is a DK part that calculates control signals to actuators based on current location
    # from GPS
    planner = Planner(goalLocation=goalLocation, steer_gain=cfg.STEERING_P_GAIN,
                        throttle_gain=cfg.THROTTLE_P_GAIN)

    # Actuators: steering and throttle
    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM,
                                    right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    # add threaded part for gps controller
    V.add(gps, outputs=["currLocation", "prevLocation"], threaded=True)

    # add planner, actuator parts
    V.add(planner, inputs=["currLocation", "prevLocation"], outputs=["steer_cmd", "throttle_cmd"])
    V.add(steering, inputs=['steer_cmd'])
    V.add(throttle, inputs=['throttle_cmd'])

    V.start()


if __name__ == '__main__':
    # goalLocation is a list of lists: each sublist a waypoint for the controller.
    goalLocation = [[32.8811271,-117.2342783], [32.8812414, -117.2374792]]
    goalLocation = [[32.881322,-117.235454], [32.881162,-117.235459]]
    goalLocation = [[32.881018, -117.235807]]    
    cfg = dk.load_config()  
    drive(cfg, goalLocation)
