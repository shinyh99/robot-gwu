#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from scout_msgs.msg import ScoutStatus
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from math import sqrt, cos, sin, atan2
from tf import transformations

from geometry_msgs.msg import Twist

import numpy as np


class ScoutController:
    def __init__(self) -> None:
        rospy.init_node("scout_controller", anonymous=True)

        self.is_pose = False
        rospy.Subscriber("/odom", Odometry, self.current_pose_callback)

        self.is_path = False
        rospy.Subscriber("/local_path", Path, self.path_callback)

        rospy.Subscriber("/scout_status", ScoutStatus, self.status_callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        scout_cmd_vel_msg = Twist()

        # User-Defined Length for offest
        lenth_offset = 3

        freq = 20
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            # print(self.is_pose, self.is_path)
            if self.is_pose and self.is_path:
                # print("wow")
                # Find the x_l, y_l
                length, i = 0, 0
                while length < lenth_offset and i < len(self.path.poses) - 1:
                    pose_i = self.path.poses[i]
                    i += 1
                    pose_f = self.path.poses[i]

                    length += sqrt(
                        (pose_f.pose.position.x - pose_i.pose.position.x) ** 2
                        + (pose_f.pose.position.y - pose_i.pose.position.y) ** 2
                    )

                x_l, y_l = pose_f.pose.position.x, pose_f.pose.position.y

                (_, _, heading) = transformations.euler_from_quaternion(
                    [
                        self.pose.pose.orientation.x,
                        self.pose.pose.orientation.y,
                        self.pose.pose.orientation.z,
                        self.pose.pose.orientation.w,
                    ]
                )

                translation = [
                    self.pose.pose.position.x,
                    self.pose.pose.position.y,
                ]

                t = np.array(
                    [
                        [cos(heading), -sin(heading), translation[0]],
                        [sin(heading), cos(heading), translation[1]],
                        [0, 0, 1],
                    ]
                )

                det_t = np.linalg.inv(t)
                global_x, global_y = x_l, y_l
                global_vector = [global_x, global_y, 1]
                local_vector = det_t.dot(global_vector)
                print(local_vector)

                phi = atan2(local_vector[1], local_vector[0])

                print(length, phi)

                scout_cmd_vel_msg.angular.z = phi / 10
                scout_cmd_vel_msg.linear.x = 1.0

            self.cmd_vel_pub.publish(scout_cmd_vel_msg)
            rate.sleep()

    def current_pose_callback(self, msg: Odometry):
        self.pose = msg.pose
        self.is_pose = True

    def status_callback(self, msg: ScoutStatus):
        pass
        # linear, angular velocity
        # print(msg.linear_velocity, msg.angular_velocity)

    def path_callback(self, msg: Path):
        self.path = msg
        self.is_path = True


if __name__ == "__main__":
    test = ScoutController()
