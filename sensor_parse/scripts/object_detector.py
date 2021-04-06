#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class OBDetect:
    def __init__(self):

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

        lower_object = np.array([0, 160, 90])
        upper_object = np.array([60, 240, 220])

        img_object = cv2.inRange(img_hsv, lower_object, upper_object)

        contours, _ = cv2.findContours(
            img_object, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for i, cont in enumerate(contours):

            x, y, w, h = cv2.boundingRect(cont)

            if i == 0:

                x1, y1, x2, y2 = x, y, x + w, y + h

            else:

                x1 = np.minimum(x, x1)

                y1 = np.minimum(y, y1)

                x2 = np.maximum(x + w, x2)

                y2 = np.maximum(y + h, y2)

        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)


if __name__ == "__main__":

    rospy.init_node("object_detector", anonymous=True)

    image_parser = OBDetect()

    rospy.spin()
