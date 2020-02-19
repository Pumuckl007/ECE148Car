#!/usr/bin/env python3

"""
planner.py
donkeycar part for controlling the car.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import time

class Planner():
    def __init__(self, goalLocation):

        # TODO: calibrate the throttle and steering upper and lower bounds
        self.throttle_cmd = 0.01               # throttle command to send to DC motor

        self.steering_cmd = 0                  # steer command to send to servos
        self.bearing = 0                       # current bearing error to goal [rad]

        # GPS location trackers
        self.numWaypoints = len(goalLocation)  # number of waypoints
        self.currWaypoint = 0                  # waypoint index for goalLocation list
        self.currLocation = [0, 0]             # current GPS lcoation; initialize at equator
        self.goalLocation = [[x * pi/180 for x in y] for y in goalLocation] # convert from degrees to radians
        self.distance = 100                    # tracks the distance to goal. initialize at 100m
        
        #reduced from 15m to 5m
        self.goalThreshold = 5                # the setpoint threshold for distance to goal [m]

        # initialize a text file
        #self.textFile = open('gps_data.txt', 'w')

    def run(self, stop_cmd):

        self.steer_cmd = 0
        #415 is our driving speed, 405 is our neutral
        #TODO Make these constants easier to find/change
        self.throttle_cmd = 0 if stop_cmd else 0.5
        #print("planner returning throttle of %f" % (self.throttle_cmd))
        return self.steer_cmd, self.throttle_cmd

    def shutdown(self):
        return

    def steering_controller(self, bearing_angle):
        """
        steering_controller()

        Method to implement a steering proportional controller for donkeycar
        Notes from Saurabh: Modified this to get bearing from IMU rather than
        the by using the diff be
        @params: bearing_angle - the angle from the current position to the next waypoint
        @return: steering command
        """
        #2pi radians aka 0 rads represents North. bearing angle
        #ranges from 0 to +2pi. 2pi- bearing - pi = negative if
        #we need to turn right, positive if we need to turn left
        #if we're off by a negative angle, then we compensate by changing 
        #in the positive direction, hence the - sign
        self.bearing = -(2*pi - bearing_angle - pi)
        #the steerer automatically converts an angle to pwm
        self.steer_cmd = self.steer_gain * self.bearing

        # hard limits
        if self.steering_cmd > self.steering_right:
            self.steering_cmd = self.steering_right
        elif self.steering_cmd < self.steering_left:
            self.steering_cmd = self.steering_left

        return self.steer_cmd

    def update_distance(self):
        """
        Method to update the distanc from current location to goal.
        @params: currLocation, goalLocation
        @return: distance [m]
        """
        self.distance = self.dist_between_gps_points(self.currLocation, self.goalLocation[self.currWaypoint])

        return self.distance

    def calc_bearing(self, pointA, pointB):
        """
        Method to calculate the bearing between two points A and B w.r.t. North

        @params: two gps points A and B (lat, long) (radians)
        @return: bearing from current location to goal (radians)
        """

        # extract lat and long coordinates
        lat1 = pointA[0]
        lon1 = pointA[1]
        lat2 = pointB[0]
        lon2 = pointB[1]

        diffLon = lon2 - lon1
        x = sin(diffLon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(diffLon))

        initialBearing = arctan2(x, y)

        # remap from [-pi,pi] to [0, 2*pi] for compass bearing
        compassBearingRad = (initialBearing + 2*pi) % (2*pi)

        return compassBearingRad

    def dist_between_gps_points(self, pointA, pointB):
        """
        Method to calculate the straight-line approximation between two gps coordinates.
        Used for distances on the 10-1000m scale.

        @params: two gps points A & B (radians) defined by lat and long coordinates
        @return: distance between the two points in meters
        """

        # radius of earth (m)
        r_earth = 6371e3

        # extract lat and long coordinates
        lat1 = pointA[0]
        lon1 = pointA[1]
        lat2 = pointB[0]
        lon2 = pointB[1]

        dlat = lat2 - lat1  # change in latitude
        dlon = lon2 - lon1  # change in longitude

        dx = r_earth * dlon * cos((lat1+lat2)/2)
        dy = r_earth * dlat

        dist = sqrt(square(dx)+square(dy))  # straight line approximation

        return dist

    # Edit by Sidney 3/11/2018
    def print_process(self):
        """
        Print out information like current location, distance to target,
        angle betwwen target and North,
        angle betwwen current location and North (bearing),
        the turning angle and speed
        """
        print("Goal wait counter: %d" % self.goalWaitCounter)
        print("Current waypoint: %d" % self.currWaypoint)
        print(self.goalLocation[self.currWaypoint])
        print(self.reachGoal)
        # print("Current location: [%1.8f, %1.8f]" % (self.currLocation[0], self.currLocation[1]))
        # print("Previous location: [%1.8f, %1.8f]" % (self.prevLocation[0], self.prevLocation[1]))

        # print("Bearing (rad): %f | Steering: %f" % (self.bearing, self.steer_cmd))
        print("Distance (m): %f | Throttle: %f" % (self.distance, self.throttle_cmd))
        return None
