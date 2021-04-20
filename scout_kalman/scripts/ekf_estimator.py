#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist, PoseWithCovariance
from gps_imu_parser import Converter


class ExtendedKalmanFilter:
    def __init__(self, dt=0.05) -> None:
        self.T = dt

        self.X = np.array([0, 0, 0], dtype=float).reshape([-1, 1])

        self.P = np.diag([2, 2, 2])

        self.Q = self.T * np.diag([0.1, 0.1, 0.02])

        self.R = np.diag([0.1, 0.1]) / self.T

        # use x, and y
        self.H = np.array([[1, 0, 0], [0, 1, 0]], dtype=float)

        self.pose_pub = rospy.Publisher("/pose_ekf", Odometry, queue_size=1)

        self.odom_msg = Odometry()

        self.odom_msg.header.frame_id = "/map"

    def prediction(self, u):
        dX_pre = np.zeros((3, 1), dtype=float)
        dX_pre[0, :] = u[0] * np.cos(self.X[2, :])
        dX_pre[1, :] = u[0] * np.cos(self.X[2, :])
        dX_pre[2, :] = u[1]

        self.X += self.T * dX_pre

        # chagne angle beyond limit
        if self.X[2, :] < -np.pi:
            self.X[2, :] = self.X[2, :] + 2 * np.pi
        elif self.X[2, :] > np.pi:
            self.X[2, :] = self.X[2, :] - 2 * np.pi
        else:
            pass

        self.calc_F(self.X, u)

        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q

    def correction(self, Z):
        K = self.P.dot(self.H.T).dot(
            np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R)
        )
        Y = self.H.dot(self.X)

        self.X = self.X + K.dot(Z - Y)

        self.P = self.P - K.dot(self.H).dot(self.P)

    def calc_F(self, x, u):
        self.F = np.zeros_like(self.Q, dtype=float)

        self.F[0, 2] = -u[0] * np.sin(x[2, :])
        self.F[1, 2] = u[0] * np.cos(x[2, :])

        self.F = np.identity(x.shape[0]) + self.T * self.F

    def send_estimated_state(self):
        pose_cov_ekf = PoseWithCovariance()

        q = transformations.quaternion_from_euler(0, 0, self.X[2][0])

        pose_cov_ekf.pose.position.x = self.X[0][0]
        pose_cov_ekf.pose.position.y = self.X[1][0]
        pose_cov_ekf.pose.position.z = 0.0

        pose_cov_ekf.pose.orientation.x = q[0]
        pose_cov_ekf.pose.orientation.y = q[1]
        pose_cov_ekf.pose.orientation.z = q[2]
        pose_cov_ekf.pose.orientation.w = q[3]

        P_3d = np.zeros((6, 6))
        P_3d[:2, :2] = self.P[:2, :2]
        P_3d[-1, :2] = self.P[-1, :2]
        P_3d[:2, -1] = self.P[:2, -1]
        P_3d[-1, -1] = self.P[-1, -1]

        pose_cov_ekf.covariance = P_3d.reshape([-1]).tolist()

        self.odom_msg.pose = pose_cov_ekf

        self.pose_pub.publish(self.odom_msg)

        # br = TransformBroadcaster()

        # br.sendTransform(
        #     (self.X[0][0], self.X[1][0], 0.0),
        #     transformations.quaternion_from_euler(0, 0, self.X[2][0]),
        #     rospy.Time.now(),
        #     "base_link2",
        #     "map",
        # )


class CMDPublisher:
    def __init__(self, dt: float) -> None:
        self.dt = dt
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.u = np.zeros((2,))

        self.cmd_msg = Twist()

        self.t = 0.0

    def calc_u(self):
        self.u[0] = 0.2

        self.u[1] = 0.5 * np.sin(self.t * 2 * np.pi / 5)

        self.cmd_msg.angular.z = self.u[1]

        self.cmd_msg.linear.x = self.u[0]

        self.t += self.dt

        self.cmd_pub.publish(self.cmd_msg)

        return self.u


if __name__ == "__main__":
    rospy.init_node("ekf_estimator", anonymous=True)

    freq = 20

    rate = rospy.Rate(freq)

    loc_sensor = Converter()

    ekf = ExtendedKalmanFilter(dt=1 / freq)

    cmd_gen = CMDPublisher(1 / freq)

    while not rospy.is_shutdown():
        if loc_sensor.x is not None and loc_sensor.y is not None:

            # decide the u
            u = cmd_gen.calc_u()

            # prediction step
            ekf.prediction(u)

            # measurement locations
            z = np.array([loc_sensor.x, loc_sensor.y]).reshape([-1, 1])

            # correction step
            ekf.correction(z)

            # get the estimated states
            ekf.send_estimated_state()

        else:
            pass

        rate.sleep()
