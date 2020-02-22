#!/usr/bin/env python3

"""
planner.py
donkeycar part for controlling the car.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""

from __future__ import print_function
from numpy import pi, cos, sin, arctan2, sqrt, square, radians
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
import math
import time

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
        self.needsCalc = False

    def computeDistance(self, xA, yA, xB, yB):
        h = abs(yA - yB)
        w = abs(xA - xB)
        dist = 1/h * self.distance_calibration
        print("Found person " + str(dist) + " meters away")
        return dist

    def computeAngle(self, xA, yA, xB, yB):
        xCent = xA + xB
        xCent = xCent / 2
        xCent = xCent - 200
        angle = xCent * 50/200
        print("Found person at " + str(angle) + " degreese")
        return angle

    def run(self, frame):
        self.needsCalc = False
        if not hasattr(self, 'image') or self.image is None:
                return
        # Capture frame-by-frame
        image = self.image
        # print(str(image))
        image = image.transpose([1, 0, 2])
        image = imutils.resize(image, width=min(400, 300))

        # detect people in the image
        # (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        # 	padding=(8, 8), scale=1.05)
        (rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4), padding=(8, 8), scale=1.05)

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

        minDist = 1000
        throttle = 0
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            dist = self.computeDistance(xA, yA, xB, yB)
            angle = self.computeAngle(xA, yA, xB, yB)
            if dist < minDist :
                minDist = dist
                throttle = 0.07
                self.steering_cmd = angle = angle / self.steering_max_angle

        self.out = (self.steering_cmd, throttle)
        print("Steering at " + str(self.steering_cmd))

    def update(self):
        # the function run in it's own thread
        while True:
            if(self.needsCalc):
                self.run(self.image)
            else:
                time.sleep(0.01)

    def run_threaded(self, image):
        self.image = image;
        self.needsCalc = True
        return self.out[0], self.out[1]

    def shutdown(self):
        return
