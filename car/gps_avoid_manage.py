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
from wp_parts.rtkgps import RTKGPS
from wp_parts.planner import KiwiPlanner
from wp_parts.personFinder import PersonFinder
from wp_parts.mixer import Mixer
from wp_parts.gpsheading import GPSHeading
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.controller import LocalWebController

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

    ctr = LocalWebController()

    # GPS is a DK part that will poll GPS data from serial port
    # and output current location in radians.
    gps = RTKGPS()
    personFinder = PersonFinder(steer_gain=0.5, distance_calibration=466)
    gpsHeading = GPSHeading(2)

    # Planner is a DK part that calculates control signals to actuators based on current location
    # from GPS
    planner = KiwiPlanner(goalLocation=goalLocation, steer_gain=cfg.STEERING_P_GAIN,
                        throttle_gain=cfg.THROTTLE_P_GAIN)

    # Actuators: steering and throttle
    steering_controller = PCA9685(1, 0x40, busnum=1)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=310,
                                    right_pulse=465)

    throttle_controller = PCA9685(2, 0x40, busnum=1)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=460,
                                    zero_pulse=370,
                                    min_pulse=330)

    mixer = Mixer()

    from donkeycar.parts.camera import Webcam
    cam = Webcam(image_w=400, image_h=300, image_d=3)

    V.add(cam, inputs=[], outputs=['cam/img'], threaded=True)

    V.add(personFinder, inputs=["cam/img"],
            outputs=["steer_cmd_person", "throttle_cmd_person", "proc/img"], threaded=True)

    # add threaded part for gps controller
    # We no longer need the GPS to output previous location
    V.add(gps, outputs=["currLocation", "prevLocation"], threaded=True)
    V.add(gpsHeading, inputs=["currLocation"], outputs=["heading"])

    # add planner, actuator parts
    # Previous location is no longer needed
    # Instead, use actual bearing from DMP
    # It also takes in stop_cmd, a boolean indicating whether to stop
    # in which case it reverts to "STOPPED_PWM"
    V.add(planner, inputs=["currLocation", "heading"],
            outputs=["steer_cmd_planner", "throttle_cmd_planner"])

    V.add(mixer, inputs=["steer_cmd_person", "steer_cmd_planner"],
            outputs=["steer_cmd"])

    #steer_cmd is a pwm value
    V.add(steering, inputs=['steer_cmd'])
    # throttle takes in a throttle_cmd pwm value,
    V.add(throttle, inputs=['throttle_cmd_person'])

    V.add(ctr,
          inputs=['proc/img'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    print("You can now go to <your jetson's hostname.local>:8887 to view your car's camera.")

    V.start()


if __name__ == '__main__':
    GPS_DATA = "../gps_client/data.json"
    # goalLocation is a list of lists: each sublist a waypoint for the controller.
    # with open(GPS_DATA) as f:
    #     data = json.load(f)
    # waypoints = []
    # for datum in data["markers"]:
    #     pos = datum["position"]
    #     lng = pos['lng']
    #     lat = pos['lat']
    #     waypoints.append([lng, lat])
    straight_away = [[32.881105, -117.2333215], [32.881018, -117.235807]]
    right_angle =  [[32.881165, -117.234064],
                  [32.881165, -117.234574],
                  [32.881296, -117.234574],
                  [32.881296, -117.235162],
                  [32.881034, -117.235162],
                  [32.881034, -117.235471],
                  [32.881271, -117.235471]]
    goalLocation = [[32.8811271,-117.2342783], [32.8812414, -117.2374792]]
    goalLocation = [[32.881322,-117.235454], [32.881162,-117.235459]]
    waypoints = straight_away
    print("Waypoints: {}".format(waypoints))


    cfg = dk.load_config()
    drive(cfg, waypoints)
