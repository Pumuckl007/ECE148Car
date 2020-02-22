#!/usr/bin/env python3

"""
planner.py
donkeycar part for controlling the car.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import time
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
import math

class PersonFinder():
    def __init__(self, steer_gain, distance_calibration):

        self.steer_gain = steer_gain           # TODO: add PID steering controller
        self.steering_max_angle = 100                # calibrate with DK actuator parts inputs
        self.steering_cmd = 0                  # steer command to send to servos
        self.bearing = 0                       # current bearing error to goal [rad]

        self.distance_calibration = distance_calibration

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.out = (0, 0)

    def computeDistance(xA, yA, xB, yB):
        h = abs(yA - yB)
        w = abs(xA - xB)
        dist = 1/h * calibration
        print("Found person " + str(dist) + " meters away")
        return dist

    def computeAngle(xA, yA, xB, yB):
        xCent = xA + xB
        xCent = xCent / 2
        xCent = xCent - 200
        angle = xCent * 50/200
        print("Found person at " + str(angle) + " degreese")
        return angle

    def run(self, frame):
        self.needsCalc = False
        # Capture frame-by-frame

        image = imutils.resize(self.image, width=min(400, frame.shape[1]))

        # detect people in the image
        # (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        # 	padding=(8, 8), scale=1.05)
        (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        	padding=(8, 8), scale=1.05)

        # draw the original bounding boxes
        for (x, y, w, h) in rects:
        	cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        draw = np.zeros([300, 400, 3]);

        people = []
        id = 1

        self.steer_cmd = 0

        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            dist = computeDistance(xA, yA, xB, yB)
            angle = computeAngle(xA, yA, xB, yB)
            self.steer_cmd = angle = angle / self.steering_max_angle


        return self.steer_cmd, 0.1

    def update(self):
        # the function run in it's own thread
        while True:
            if(self.needsCalc):
                self.out = self.run(self.in)
            else:
                time.sleep(0.01)

    def run_threaded(self, image):
        self.image = image;
        self.needsCalc = True
        return self.out

    def shutdown(self):
        return
