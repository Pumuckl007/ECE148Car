#!/usr/bin/env python3
"""
kiwi_manage.py

Script to control donkey car with GPS navigation along with IMU-based heading
and ultrasonic-based object detection.
Waypoints are set with GPS coordinates in degrees.

Call: gps_manage.py -drive
"""

# import GPS Planner and other DK parts
import donkeycar as dk
from wp_parts.personFinder import PersonFinder
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

# other important modules
import serial
import pynmea2
import time
import threading

#for geting the GPS coordinates
import json


def drive():

    # initialize vehicle
    V = Vehicle()

    # Planner is a DK part that calculates control signals to actuators based on current location
    # from GPS
    planner = PersonFinder(steer_gain=0.5, distance_calibration=466)

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

    from donkeycar.parts.camera import Webcam
    cam = Webcam(image_w=400, image_h=300, image_d=3)

    V.add(cam, inputs=[], outputs=['cam/img'], threaded=threaded)


    # add planner, actuator parts
    # Previous location is no longer needed
    # Instead, use actual bearing from DMP
    # It also takes in stop_cmd, a boolean indicating whether to stop
    # in which case it reverts to "STOPPED_PWM"
    V.add(planner, inputs=["cam/img"],
            outputs=["steer_cmd", "throttle_cmd"])

    #steer_cmd is a pwm value
    V.add(steering, inputs=['steer_cmd'])
    # throttle takes in a throttle_cmd pwm value,
    V.add(throttle, inputs=['throttle_cmd'])

    V.start()


if __name__ == '__main__':

    drive()
