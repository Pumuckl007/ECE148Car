#!/usr/bin/env python3
"""
gps.py
donkeycar part for interfacing with GPS. Polls GPS data and sends to Vehicle.py.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import serial
import pynmea2

from .rtkgps.gps import C099F9P


class RTKGPS():
    def __init__(self):
        # GPS coordinates used for controller
        self.currLocation = [0, 0]
        self.prevLocation = [0, 0]

        # GPS serial object
        self.gpsObj = C099F9P()
        self.gpsObj.setUpdateRate();

        self.on = True  # threading

    def poll(self):

        if self.gpsObj.newGGA:
            # update the current location of car
            self.prevLocation = self.currLocation
            currLocation = self.GPStoRad(self.gpsObj.gga)
            self.currLocation = currLocation
            # print(self.currLocation)
        else:
            # do nothing; don't need the other serial data
            pass
        return

    def update(self):
        while self.on:
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
