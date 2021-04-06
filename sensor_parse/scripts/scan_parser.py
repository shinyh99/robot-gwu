#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32



class SCANParser:

    def __init__(self):

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)

        self.dist_pub = rospy.Publisher("dist_forward", Float32, queue_size=10)

    def calc_dist_forward(self):

        r_forward1 = self.R[:30]

        r_forward2 = self.R[-30:]

        r_forward = np.concatenate([r_forward1, r_forward2], axis=0)

        return np.min(r_forward)

    def callback(self, msg):

        self.R = np.array(msg.ranges)

        d_min = self.calc_dist_forward()

        dist_msg = Float32()

        dist_msg.data = d_min

        self.dist_pub.publish(dist_msg)


if __name__ == '__main__':

    rospy.init_node('scan_parser', anonymous=True)

    scan_parser = SCANParser()

    rospy.spin() 
