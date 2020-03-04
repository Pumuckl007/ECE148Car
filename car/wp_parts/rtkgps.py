#!/usr/bin/env python3
"""
gps.py
donkeycar part for interfacing with GPS. Polls GPS data and sends to Vehicle.py.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import serial
import pynmea2

from .librtkgps.gps import C099F9P


class RTKGPS():
    def __init__(self):
        # GPS coordinates used for controller
        self.currLocation = [0, 0]
        self.prevLocation = [0, 0]

        # GPS serial object
        self.gpsObj = C099F9P(ip="", callback=self.newData)
        self.gpsObj.setUpdateRate();

        self.on = True  # threading

    def poll(self):
        self.gpsObj.read()

    def newData(self):
        if self.gpsObj.newGGA:
            # update the current location of car
            self.prevLocation = self.currLocation
            currLocation = self.gpsObj.latlon
            currLocation = (currLocation[0] * pi/180, -currLocation[1]*pi/180)
            #print(self.gpsObj.gga)
            #print("CANT MISS THIS")
            self.currLocation = currLocation
            self.gpsObj.newGGA = False

    def update(self):
        while True:
            self.poll()

    def run_threaded(self):
        return (self.currLocation, self.prevLocation)

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('Stopping GPS...')

    def GPStoRad(self, gpsData):
        """
        GPStoRad(gpsData)

        Method to parse GPS data and determine angular positon using the pynmea2 library.
        @params: gpsData in GPGGA format
        @return: gpsFloat tuple (latitude, longditude) in radians
        """
        nmeaObj = pynmea2.parse(gpsData)  # create nmea object
        lat = radians(nmeaObj.latitude)  # degrees->radians
        lon = radians(nmeaObj.longitude)  # degrees->radians
        gpsFloat = (float(lat), float(lon))

        return gpsFloat
