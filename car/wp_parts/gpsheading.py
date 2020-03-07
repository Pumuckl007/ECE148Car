#!/usr/bin/env python3

"""
gpsheading.py
donkeycar part for calculating heading from GPS data

@authors: Max Apodaca, Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import time

class GPSHeading():
    def __init__(self, positionWindow = 3):
        self.recentMeasurements = []
        self.prevLocation = (0,0)
        self.previousHeading = 0
        self.positionWindow = positionWindow

    def run(self, currLocation):
        if currLocation == self.prevLocation :
            return self.previousHeading

        self.prevLocation = currLocation
        self.recentMeasurements.append(currLocation)

        if len(self.recentMeasurements) < 2 :
            return self.previousHeading

        if len(self.recentMeasurements) > (self.positionWindow*2) :
            self.recentMeasurements.pop(0)

        centroids = self.compute_centroids(self.recentMeasurements)

        #print("centroids " + str(centroids))

        bearing = self.calc_bearing(centroids[1], centroids[0])
        self.previousHeading = bearing
        return bearing

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

        return initialBearing
