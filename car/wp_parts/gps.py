#!/usr/bin/env python3
"""
gps.py
donkeycar part for interfacing with GPS. Polls GPS data and sends to Vehicle.py.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import serial
import pynmea2


class GPS():
    def __init__(self, baudRate, port, timeOut):
        # GPS coordinates used for controller
        self.currLocation = [0, 0]

        # GPS serial object
        self.gpsObj = serial.Serial(
        	port     = port,
        	baudrate = baudRate,
        	timeout  = timeOut,
        	parity   = serial.PARITY_NONE,
        	stopbits = serial.STOPBITS_ONE,
        	rtscts   = False,
        	xonxoff  = False,
        	dsrdtr   = False)

        self.on = True  # threading

    def poll(self):
        """
        Sidney's GPS polling function
        Method to poll gps data and parse using GPStoRad
        """
        gpsdataByte = self.gpsObj.readline()
        gpsdata = gpsdataByte.decode("utf-8")  # convert from byte to string

        identifier = "$GPGGA"  # identifier for relevant GPS NMEA data

        if identifier in gpsdata:
            # update the current location of car
            self.prevLocation = self.currLocation
            currLocation = self.GPStoRad(gpsdata)
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
        return self.currLocation

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
