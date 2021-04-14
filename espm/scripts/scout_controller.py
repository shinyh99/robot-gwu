#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from std_msgs.msg import Float32

from geometry_msgs.msg import Point, Twist
from scout_msgs.msg import ScoutStatus
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from simple_pid import PID


class FollowTheCarrot:
    def __init__(self):
        rospy.init_node("FollowTheCarrot", anonymous=True)

        # Local path
        self.path_on = False
        rospy.Subscriber("local_path", Path, self.path_callback)

        # Odometry
        self.odom_on = False
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Scout status
        self.speed_current_on = False
        # self.time = 0.0
        rospy.Subscriber("/scout_status", ScoutStatus, self.status_callback)

        # Longitudinal velocity PID control
        self.speed_target_on = False
        self.pid = PID(Kp=0.2, Ki=1, Kd=0.0, output_limits=(-100, 100))
        rospy.Subscriber("/target_velocity", Float32, self.velocity_callback)

        # Lateral control
        self.ctrl_msg = Twist()
        self.ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/status_vel", Float32, queue_size=1)

        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.lfd_ratio = 0.5
        self.lfd = 3.0
        self.min_lfd = 3.0
        self.max_lfd = 30.0

        freq = 20
        self.pid.sample_time = 1 / freq
        rate = rospy.Rate(freq)  # 20hz
        while not rospy.is_shutdown():

            if self.path_on and self.odom_on and self.speed_current_on:

                ## Longitudinal control
                if self.speed_target_on:
                    self.speed_input = self.pid(self.speed_current)
                    # self.speed_input = 3.0
                    print(self.speed_input)
                else:
                    self.speed_input = 3.0

                ## Lateral control
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
                    self.ctrl_msg.linear.x = self.speed_input

                else:
                    # 전방주시 포인트를 찾지 못했을 때
                    self.ctrl_msg.angular.z = 0.0
                    self.ctrl_msg.linear.x = 0.0

                self.ctrl_pub.publish(self.ctrl_msg)
                self.status_pub.publish(self.speed_current)

            rate.sleep()

    def path_callback(self, msg: Path):
        self.path_on = True
        self.path = msg  # nav_msgs/Path

    def odom_callback(self, msg: Odometry):
        self.odom_on = True
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

    def velocity_callback(self, msg: Float32):
        self.pid.setpoint = msg.data
        self.speed_target_on = True

    def status_callback(self, msg: ScoutStatus):
        self.speed_current = msg.linear_velocity
        self.speed_current_on = True


if __name__ == "__main__":

    test = FollowTheCarrot()
