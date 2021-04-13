#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class followTheCarrot:
    def __init__(self):
        rospy.init_node("followTheCarrot", anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.ctrl_msg = Twist()
        self.is_path = False
        self.is_odom = False

        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.lfd_ratio = 0.5
        self.lfd = 3.0
        self.min_lfd = 3.0
        self.max_lfd = 30.0

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True:

                vehicle_position = self.current_postion
                rotated_point = Point()
                self.is_look_forward_point = False

                translation = [vehicle_position.x, vehicle_position.y]

                t = np.array(
                    [
                        [
                            cos(self.vehicle_yaw),
                            -sin(self.vehicle_yaw),
                            translation[0],
                        ],
                        [
                            sin(self.vehicle_yaw),
                            cos(self.vehicle_yaw),
                            translation[1],
                        ],
                        [0, 0, 1],
                    ]
                )

                det_t = np.array(
                    [
                        [
                            t[0][0],
                            t[1][0],
                            -(
                                t[0][0] * translation[0]
                                + t[1][0] * translation[1]
                            ),
                        ],
                        [
                            t[0][1],
                            t[1][1],
                            -(
                                t[0][1] * translation[0]
                                + t[1][1] * translation[1]
                            ),
                        ],
                        [0, 0, 1],
                    ]
                )

                # self.lfd=self.current_velocity*self.lfd_ratio
                # if self.lfd < self.min_lfd :
                #     self.lfd=self.min_lfd
                # elif self.lfd > self.max_lfd :
                #     self.lfd=self.max_lfd

                min_dis = float("inf")
                for num, i in enumerate(self.path.poses):
                    path_point = i.pose.position

                    dis = sqrt(
                        pow(
                            path_point.x - self.path.poses[0].pose.position.x, 2
                        )
                        + pow(
                            path_point.y - self.path.poses[0].pose.position.y, 2
                        )
                    )
                    # print(dis)
                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = path_point
                        self.is_look_forward_point = True

                if self.is_look_forward_point:
                    global_path_point = [
                        self.forward_point.x,
                        self.forward_point.y,
                        1,
                    ]
                    local_path_point = det_t.dot(global_path_point)
                    theta = atan2(local_path_point[1], local_path_point[0])
                    self.ctrl_msg.angular.z = -theta
                    self.ctrl_msg.linear.x = 3.0

                else:
                    # 전방주시 포인트를 찾지 못했을 때
                    self.ctrl_msg.angular.z = 0.0
                    self.ctrl_msg.linear.x = 0.0

                self.ctrl_pub.publish(self.ctrl_msg)

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg  # nav_msgs/Path

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
        self.current_velocity = msg.twist.twist.linear.x


if __name__ == "__main__":

    test = followTheCarrot()
