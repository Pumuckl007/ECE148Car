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
from wp_parts.gps import GPS
from wp_parts.ultrasonic import HCSR04
from wp_parts.dmp import MPU9265
from wp_parts.planner import KiwiPlanner
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

# other important modules
import serial
import pynmea2
import time
import threading

#for geting the GPS coordinates
import json


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
    dmp = MPU9265()

    # Planner is a DK part that calculates control signals to actuators based on current location
    # from GPS
    planner = KiwiPlanner(goalLocation=goalLocation, steer_gain=cfg.STEERING_P_GAIN,
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
    # We no longer need the GPS to output previous location
    V.add(gps, outputs=["currLocation"], threaded=True)
    
    #the DMP in the IMU should return the bearing relative to North
    # TODO - implement this part...
    V.add(dmp, outputs=["bearing"], threaded=True)

    #the ultrasonics will tell you whether you need to stop
    #True means stop, False means go
    # This part should be good to go - Saurab
    ultrasonic = HCSR04()
    V.add(ultrasonic, outputs=['stop_cmd'], threaded=True)

    # add planner, actuator parts
    # Previous location is no longer needed
    # Instead, use actual bearing from DMP
    # It also takes in stop_cmd, a boolean indicating whether to stop
    # in which case it reverts to "STOPPED_PWM"
    V.add(planner, inputs=["currLocation", "bearing", "stop_cmd"], 
            outputs=["steer_cmd", "throttle_cmd"])

    #steer_cmd is a pwm value
    V.add(steering, inputs=['steer_cmd'])
    # throttle takes in a throttle_cmd pwm value,
    V.add(throttle, inputs=['throttle_cmd'])

    V.start()


if __name__ == '__main__':
    GPS_DATA = "../gps_client/data.json"
    # goalLocation is a list of lists: each sublist a waypoint for the controller.
    with open(GPS_DATA) as f:
        data = json.load(f)
    waypoints = []
    for datum in data["markers"]:
        pos = datum["position"]
        lng = pos['lng']
        lat = pos['lat']
        waypoints.append([lng, lat])
    straight_away = [[32.881017, -117.234106], [32.881018, -117.235807]]
    right_angle =  [[32.881165, -117.234064],
                  [32.881165, -117.234574], 
                  [32.881296, -117.234574],
                  [32.881296, -117.235162],
                  [32.881034, -117.235162],
                  [32.881034, -117.235471],
                  [32.881271, -117.235471]]
    goalLocation = [[32.8811271,-117.2342783], [32.8812414, -117.2374792]]
    goalLocation = [[32.881322,-117.235454], [32.881162,-117.235459]]
    waypoints = straight_away[1:2]
    print("Waypoints: {}".format(waypoints))
    

    cfg = dk.load_config()  
    drive(cfg, waypoints)
