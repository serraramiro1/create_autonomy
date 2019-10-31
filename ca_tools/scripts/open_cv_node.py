#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class image_converter:

    def __init__(self):
        self.is_red_pub = rospy.Publisher("isred", Bool, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/create1/raspicam/image_raw", Image, self.callback)
        self.cv_image = None
        self.img_to_detect = cv2.imread(rospy.get_param('img_url'))
        self.img_to_detect = cv2.cvtColor(
            self.img_to_detect, cv2.COLOR_BGR2GRAY)

    def convert(self):

        MIN_MATCH_COUNT = 50
        img2 = self.cv_image

        if (img2 is None):
            return

        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        w, h = self.img_to_detect.shape

        surf = cv2.xfeatures2d.SURF_create(
            400, nOctaves=3, nOctaveLayers=4, extended=False, upright=True)
        # find the keypoints and descriptors with SURF
        kp1, des1 = surf.detectAndCompute(self.img_to_detect, None)
        kp2, des2 = surf.detectAndCompute(img2, None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        try:
            matches = flann.knnMatch(des1, des2, k=2)
        except:
            return

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m, n in matches:
            if m.distance < 0.8 * n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.uint8(
                [kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.uint8(
                [kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if M is None:
                return
            if mask is None:
                return

            matchesMask = mask.ravel().tolist()

            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                              [w - 1, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, M)
            self.is_red_pub.publish(True)
            img2 = cv2.polylines(
                img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA).astype(np.uint8)

        else:
            rospy.loginfo("Not enough matches are found - %d/%d",
                          len(good), MIN_MATCH_COUNT)
            self.is_red_pub.publish(False)
            matchesMask = None

        draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                           singlePointColor=None,
                           matchesMask=matchesMask,  # draw only inliers
                           flags=2)

        img3 = np.uint8(cv2.drawMatches(self.img_to_detect, kp1,
                                        img2, kp2, good, None, **draw_params))

        cv2.imshow("imagecomparison", img3)
        cv2.waitKey(50)

    def callback(self, data):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    aux = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        ic.convert()
        aux.sleep()
