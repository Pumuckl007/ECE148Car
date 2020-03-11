#!/usr/bin/env python3

"""
speedometer.py
donkeycar part for calculating speed from GPS data

@authors: Max Apodaca
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import time

class Speedometer():
    def __init__(self, speedPID, positionWindow = 3, updatePeriod=0.05):
        self.recentMeasurements = []
        self.prevLocation = (0,0)
        self.previousSpeed = 0
        self.updatePeriod = updatePeriod
        self.positionWindow = positionWindow
        self.speedPID = speedPID

    def run(self, currLocation):
        if currLocation == self.prevLocation :
            return self.previousSpeed

        self.prevLocation = currLocation
        self.recentMeasurements.append(currLocation)

        if len(self.recentMeasurements) < 2 :
            return self.previousSpeed

        if len(self.recentMeasurements) > (self.positionWindow*2) :
            self.recentMeasurements.pop(0)

        centroids = self.compute_centroids(self.recentMeasurements)

        #print("centroids " + str(centroids))

        distance = self.dist_between_gps_points(centroids[1], centroids[0])
        time = self.updatePeriod * self.positionWindow
        speed = distance / time
        self.previousSpeed = speed
        self.speedPID.sample(speed)
        return speed

    def compute_centroids(self, locations):
        locationCount = len(locations)
        middleIndex = int(locationCount / 2)
        oldLocations = locations[:middleIndex]
        newLocations = locations[middleIndex:]
        oldCentroid = self.compute_centroid(oldLocations)
        newCentroid = self.compute_centroid(newLocations)
        return [oldCentroid, newCentroid]

    def compute_centroid(self, locations):
        averageLat = 0
        averageLon = 0
        for location in locations:
            averageLat = averageLat + location[0]
            averageLon = averageLon + location[1]
        return (averageLat / len(locations), averageLon / len(locations))

    def shutdown(self):
        return

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
