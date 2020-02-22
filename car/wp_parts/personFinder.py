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
        self.steering_max_angle = 50                # calibrate with DK actuator parts inputs
        self.steering_cmd = 0                  # steer command to send to servos
        self.bearing = 0                       # current bearing error to goal [rad]

        self.distance_calibration = distance_calibration

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.out = (0, 0, 0)
        self.needsCalc = False
        self.throttle = 0

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

    def drawOnImage(self, img, dist, angle, id=0):
        # x = dist * math.sin(angle*3.14/180) * 300/5 + 200
        # y = 300 - (dist * math.cos(angle*3.14/180) * 300/5)
        # x = angle*200/50 + 200
        x = int(angle*200/50 + 200)
        cv2.rectangle(img, (x-2, 0), (x+2, 300), (0, 0, 255), 2)

        # cv2.ellipse(img, (int(x), int(y)), (60, 60), 0.0, 0.0, 360.0, (255, 1-id/3, id/3), -1)

    def run(self, frame):
        self.needsCalc = False
        if not hasattr(self, 'image') or self.image is None:
                return
        # Capture frame-by-frame
        image = self.image
        # print(str(image))
        image = image.transpose([1, 0, 2])
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        image = imutils.resize(image, width=400)
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
        self.throttle = self.throttle / 1.1
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
            dist = self.computeDistance(xA, yA, xB, yB)
            angle = self.computeAngle(xA, yA, xB, yB)
            self.drawOnImage(image, dist, angle)
            if dist < minDist :
                minDist = dist
                self.throttle = 0.08
                self.steering_cmd = -angle / self.steering_max_angle

        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        imageOut = image.transpose([1, 0, 2])
        self.out = (self.steering_cmd, 0.08, imageOut)
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
        return self.out[0], self.out[1], self.out[2]

    def shutdown(self):
        return
