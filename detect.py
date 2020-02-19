from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
import math

calibration = 446

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

def drawOnImage(img, dist, angle, id=0):
    x = dist * math.sin(angle*3.14/180) * 300/5 + 200
    y = 300 - (dist * math.cos(angle*3.14/180) * 300/5)
    cv2.ellipse(img, (int(x), int(y)), (60, 60), 0.0, 0.0, 360.0, (255, 1-id/3, id/3), -1)

def mainLoop():

    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cap = cv2.VideoCapture(0)


    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        image = imutils.resize(frame, width=min(400, frame.shape[1]))
        orig = image.copy()

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

        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
            dist = computeDistance(xA, yA, xB, yB)
            angle = computeAngle(xA, yA, xB, yB)
            drawOnImage(draw, dist, angle, id)
            id = id + 1

        cv2.ellipse(draw, (200, 300), (5, 5), 0.0, 0.0, 360.0, (255, 0, 0), -1)

        cv2.imshow("After NMS", image)
        cv2.imshow("Map", draw)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
mainLoop()
