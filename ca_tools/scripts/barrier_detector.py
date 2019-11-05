#!/usr/bin/env python
"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
"""
import sys
import math
import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class barrier_detector(object):

    def __init__(self):
        self.is_barrier_pub = rospy.Publisher(
            "/isbarrier", Bool, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/create1/raspicam/image_raw", Image, self.callback)
        self.cv_image = None
        
    def process(self):
        count = 0
        # Loads an image
        theta = []
        # Check if image is loaded fine
        if self.cv_image is None:
            return

        dst = cv.Canny(self.cv_image, 50, 200, None, 3)

        # Copy edges to the images that will display the results in BGR
        cdstP = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)

        linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 100, None, 50, 50)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                theta.append(math.atan((l[1] - l[0])/(l[3] - l[2])))
                cv.line(cdstP, (l[0], l[1]), (l[2], l[3]),
                        (0, 0, 255), 3, cv.LINE_AA)

        for x in theta:
            if (abs(x) < 0.05):
                count += 1

        self.is_barrier_pub.publish(count >= 10)
        # print(count)
        #cv.imshow("Source", self.cv_image)
        #cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        # cv.waitKey(50)
        return 0

    def callback(self, data):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
            w, h, _ = self.cv_image.shape
            self.cv_image = self.cv_image[ h / 2:h, 0:w]

        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node('barrier_detector', anonymous=True)
    bd = barrier_detector()
    aux = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        bd.process()
        aux.sleep()
