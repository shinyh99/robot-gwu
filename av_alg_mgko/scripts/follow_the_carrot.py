#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ca_msgs.msg import CollisionAvoidance
import numpy as np


class FollowTheCarrot:
    def __init__(self):
        rospy.init_node("followTheCarrot", anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/target_vel", Float32, self.target_vel_callback)
        rospy.Subscriber(
            "/ctrl_collision", CollisionAvoidance, self.collision_callback
        )
        self.ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.ctrl_msg = Twist()
        self.is_path = False
        self.is_odom = False
        self.is_target_vel = False
        self.is_ca = False

        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.lfd_ratio = 0.5
        self.lfd = 1.5
        self.min_lfd = 3.0
        self.max_lfd = 30.0

        self.collision_data = CollisionAvoidance()

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            rate.sleep()

            if self.is_path and self.is_odom and self.is_target_vel:

                vehicle_position = self.current_postion
                # rotated_point = Point()
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

                if len(self.path.poses) > 1:
                    # Find a specific point which is "self.lfd" away from ego on the route
                    min_dis = float("inf")
                    for i in self.path.poses:
                        path_point = i.pose.position

                        dis = sqrt(
                            pow(
                                path_point.x
                                - self.path.poses[0].pose.position.x,
                                2,
                            )
                            + pow(
                                path_point.y
                                - self.path.poses[0].pose.position.y,
                                2,
                            )
                        )
                        if abs(self.lfd - dis) < min_dis:
                            min_dis = abs(self.lfd - dis)
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

                    phi_final = 0
                    d_min = 0
                    if self.is_ca:
                        alpha = self.collision_data.ca_const_alpha
                        beta = self.collision_data.ca_const_beta
                        d_min = self.collision_data.ca_distance

                        d_min_factor = 2.5 * self.target_vel

                        if d_min < d_min_factor:
                            d_min = d_min_factor * pow(
                                1 / 2,
                                d_min_factor - d_min,
                            )

                        if d_min == 0.0 or (alpha == 0.0 and beta == 0.0):
                            phi_final = theta
                        else:
                            frac_alpha_dmin = alpha / d_min
                            phi_final = (
                                frac_alpha_dmin * self.collision_data.phi_gap
                                + beta * theta
                            ) / (frac_alpha_dmin + beta)

                    else:
                        phi_final = theta

                    self.ctrl_msg.angular.z = phi_final
                    distnace_min = "%2.2f" % self.collision_data.ca_distance
                    distnace_min_adjusted = "%2.2f" % d_min
                    velocity = "%2.2f" % self.target_vel
                    print(
                        f"vel:{velocity}  dmin:{distnace_min}  adjusted: {distnace_min_adjusted}  ca: {int(self.collision_data.phi_gap * 180 / pi)}\t th: {int(theta * 180 / pi)}\t dir: {int(phi_final * 180 /pi)}"
                    )
                    self.ctrl_msg.linear.x = self.target_vel

                else:
                    # 전방주시 포인트를 찾지 못했을 때
                    self.ctrl_msg.angular.z = 0.0
                    self.ctrl_msg.linear.x = 0.0

                if self.ctrl_msg.linear.x < 0.0:
                    self.ctrl_msg.angular.z = 0.0
                    self.ctrl_msg.linear.x = 0.0
                self.ctrl_pub.publish(self.ctrl_msg)

            else:
                print(
                    f"local:{self.is_path}, odom:{self.is_odom}, target:{self.is_target_vel}",
                    flush=True,
                )

    def path_callback(self, msg: Path):
        self.is_path = True
        self.path = msg  # nav_msgs/Path

    def odom_callback(self, msg: Odometry):
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

    def target_vel_callback(self, msg: Float32):
        self.is_target_vel = True
        # self.target_vel = msg.data
        self.target_vel = 3.0

    def collision_callback(self, msg: CollisionAvoidance):
        self.is_ca = True
        self.collision_data = msg


if __name__ == "__main__":

    test = FollowTheCarrot()
