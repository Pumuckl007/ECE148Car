#!/usr/bin/env python3

"""
planner.py
donkeycar part for controlling the car.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import time

class Planner():
    def __init__(self, goalLocation, steer_gain, throttle_gain):

        # TODO: calibrate the throttle and steering upper and lower bounds

        # Throttle controller
        self.throttle_gain = throttle_gain     # TODO: tune P throttle controller
        self.throttle_lower = 0                # calibrate with DK actuator parts inputs
        self.throttle_upper = 0.14             # calibrate with DK actuator parts inputs
        self.throttle_cmd = 0.01               # throttle command to send to DC motor

        # Steering controller
        self.steer_gain = steer_gain           # TODO: add PID steering controller
        self.steering_left = -1                # calibrate with DK actuator parts inputs
        self.steering_right = 1                # calibrate with DK actuator parts inputs
        self.steering_cmd = 0                  # steer command to send to servos
        self.bearing = 0                       # current bearing error to goal [rad]

        # GPS location trackers
        self.numWaypoints = len(goalLocation)  # number of waypoints
        self.currWaypoint = 0                  # waypoint index for goalLocation list
        self.currLocation = [0, 0]             # current GPS lcoation; initialize at equator
        self.prevLocation = [0, 0]             # previous time step GPS location; initialize at equator
        self.goalLocation = [[x * pi/180 for x in y] for y in goalLocation] # convert from degrees to radians
        self.distance = 100                    # tracks the distance to goal. initialize at 100m
        self.reachGoal = False                 # flag to signify whether or not the goal was reached
        self.goalThreshold = 15                # the setpoint threshold for distance to goal [m]
        self.goalWaitCounter = 0               # counter for keeping track of waiting between waypoints
        self.goalWaitCounterThreshold = 50     # maximum wait (counts) between waypoints

        # initialize a text file
        self.textFile = open('gps_data.txt', 'w')

    def run(self, currLocation, prevLocation):

        # update the current and previous location from GPS part
        self.currLocation = currLocation
        self.prevLocation = prevLocation

        # update the distance to goal
        self.distance = self.update_distance()

        # if the distance to goal reaches a threshold value, enter waypoint routine (go in circle)
        if self.distance <= self.goalThreshold:# or self.reachGoal == True:
            self.reachGoal = True
            print("Reached waypoint!!")

            # if the car has reached the goal, drive in circle
            self.throttle_cmd, self.steer_cmd = self.drive_in_circle()

            # while driving in circle, increment a counter
            self.goalWaitCounter += 1

            # once counter reaches a threshold value, reset reachGoal flag
            # and continue on waypoints list
            if self.goalWaitCounter > self.goalWaitCounterThreshold:
                self.reachGoal = False
                self.currWaypoint += 1
                self.goalWaitCounter = 0
                self.distance = 100  # reset distance to an arbitrary distance

        else:
            # calculate steering and throttle as using controller
            self.reachGoal = False
            self.steer_cmd = self.steering_controller(currLocation, prevLocation)
            self.throttle_cmd, self.reachGoal = self.throttle_controller(currLocation, prevLocation)

        # print updates
        self.print_process()

        # write to file
        self.textFile.write("%s, %s;\n" % (self.currLocation[0], self.currLocation[1]))

        # end
        if self.currWaypoint == self.numWaypoints and self.reachGoal == True:
            print("Done.")

        return self.steer_cmd, self.throttle_cmd

    def shutdown(self):
        self.textFile.close()
        pass
        return

    def steering_controller(self, currLocation, prevLocation):
        """
        steering_controller()

        Method to implement a steering proportional controller for donkeycar
        @params: bearing_setpoint, bearing_current
        @return: steering command
        """
        # TODO: add PID steering controller
        # Proportional controller
        bearingPrevToCurr = self.calc_bearing(self.prevLocation, self.currLocation)
        bearingCurrToGoal = self.calc_bearing(self.currLocation, self.goalLocation[self.currWaypoint])
        self.bearing = bearingCurrToGoal - bearingPrevToCurr
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

    def throttle_controller(self, currLocation, prevLocation):
        """
        Method to implement a proportional controller for throttle for donkey car
        @params:
        @return:
        """

        # Proportional control
        self.throttle_cmd = self.throttle_gain * self.distance

        # hard limits
        if self.throttle_cmd > self.throttle_upper:
            self.throttle_cmd = self.throttle_upper
        elif self.throttle_cmd < self.throttle_lower:
            self.throttle_cmd = self.throttle_lower

        return self.throttle_cmd, self.reachGoal

    def drive_in_circle(self):
        """
        Method to send commands to drive the car in a circle.

        @params: self
        @return: throttle command, steering command
        """
        # go in a circle to the left at 0.75*max speed
        self.throttle_cmd = self.throttle_upper
        self.steer_cmd = -1
        return self.throttle_cmd, self.steer_cmd

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



# Methods useful for implementing planning algorithim
def haversine(pointA, pointB):
    """
    Method to calculate the (great circle) distance over a sphere with the haversine formula.
    Best for distances >1km. Otherwise computationally excessive.
    @params: two gps coordinates pointA and pointB (radians) defined by lat and long coordinates
    @return: distance between the two points in meters
    """
    if (type(pointA) != list) or (type(pointB) != list):
        raise TypeError("Only lists are supported as arguments")

    # radius of earth (m)
    r_earth = 6371e3

    # extract lat and long coordinates
    lat1 = pointA[0]
    lon1 = pointA[1]
    lat2 = pointB[0]
    lon2 = pointB[1]

    dlat = float(lat2) - float(lat1)  # change in latitude
    dlon = float(lon2) - float(lon1)  # change in longitude

    # distance over a sphere using the Haversine formula
    # https://www.movable-type.co.uk/scripts/latlong.html
    a = square(sin(dlat/2)) + cos(lat1)*cos(lat2)*square(sin(dlon/2))
    c = 2*arctan2(sqrt(a), sqrt(1-a))
    d = r_earth * c

    return d
