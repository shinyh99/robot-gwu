#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class IMGParser:
    def __init__(self):
        rospy.init_node("image_parse", anonymous=True)

    def run(self):
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.callback
        )

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        debug = 0

        if debug:
            img_concat = np.concatenate([img_bgr, img_hsv], axis=1)
        else:
            # hsv -> vsh reversed
            lower_wlane = np.array([0, 0, 200])
            upper_wlane = np.array([30, 10, 255])

            lower_ylane = np.array([0, 200, 230])
            upper_ylane = np.array([50, 255, 255])

            img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
            img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
            img_wy_lane = img_wlane + img_ylane
            # img_wy_lane = cv2.add(img_wlane, img_ylane)
            # img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)
            # img_ylane = cv2.cvtColor(img_ylane, cv2.COLOR_GRAY2BGR)

            img_wy_lane = cv2.cvtColor(img_wy_lane, cv2.COLOR_GRAY2BGR)

            img_concat = np.concatenate([img_bgr, img_hsv, img_wy_lane], axis=1)

        cv2.imshow("Image Window", img_concat)
        cv2.waitKey(1)


if __name__ == "__main__":

    image_parser = IMGParser()
    image_parser.run()

    rospy.spin()
