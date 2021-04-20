#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from ekf_estimator import CMDPublisher
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from tf import transformations
from gps_imu_parser import Converter


def pdf_multivariate_gauss(dx, cov) -> np.ndarray:
    part2 = np.exp(-0.5 * np.matmul(np.matmul(dx.T, np.linalg.inv(cov)), dx))

    part1 = 1 / (
        (2 * np.pi) ** (cov.shape[0] / 2) * (np.linalg.det(cov) ** (1 / 2))
    )

    return np.diag(part1 * part2) + 0.00001


class ParticleFilter:
    def __init__(self, dt=0.05, NP=1000) -> None:
        self.T = dt
        self.NP = NP
        self.XP = np.random.randn(3, self.NP) * 50 + np.array(
            [20, 1100.0, 0]
        ).reshape([3, 1])

        self.limit_yaw()

        self.pw = np.ones((NP,)) / NP

        self.q = [0.01, 0.01, 0.02]

        self.R = np.diag([0.1, 0.1]) / self.T

        self.H = np.array([[1, 0, 0], [0, 1, 0]], dtype=float)

        self.pose_pub = rospy.Publisher("/pose_pf", PoseArray, queue_size=1)

    def prediction(self, u):

        dX_pre = np.zeros((3, self.NP), dtype=float)
        dX_pre[0, :] = u[0] * np.cos(self.XP[2, :]) + self.q[
            0
        ] * np.random.randn(1, self.NP)
        dX_pre[1, :] = u[0] * np.sin(self.XP[2, :]) + self.q[
            1
        ] * np.random.randn(1, self.NP)
        dX_pre[2, :] = u[1] + self.q[2] * np.random.randn(1, self.NP)

        self.XP += self.T * dX_pre

        self.limit_yaw()

    def limit_yaw(self):
        over_bool = self.XP[2, :] < -np.pi

        self.XP[2, over_bool] = self.XP[2, over_bool] + 2 * np.pi

        under_bool = self.XP[2, :] > np.pi

        self.XP[2, under_bool] = self.XP[2, under_bool] - 2 * np.pi

    def correction(self, Z):
        dz = self.H.dot(self.XP) - Z

        pdf_z = pdf_multivariate_gauss(dz, self.R)

        self.pw = pdf_z / pdf_z.sum()

        self.Xm = np.dot(self.XP, self.pw).reshape([-1, 1])

        self.Xcov = np.dot((self.XP - self.Xm), np.diag(self.pw)).dot(
            (self.XP - self.Xm).T
        )

        re_sample_ID, self.pw = self.re_sampling(self.pw)

        XP_new = self.XP[:, re_sample_ID]

        self.XP = XP_new + np.diag([0.1, 0.1, 0.01]).dot(
            np.random.randn(3, self.NP)
        )
        self.limit_yaw()

    def re_sampling(self, w):
        re_sample_ID = np.random.choice(np.arange(self.NP), self.NP, p=w)

        w = np.ones((self.NP,)) / self.NP

        return re_sample_ID, w

    def send_estimated_state(self):
        particles_msg = PoseArray()
        particles_msg.header.frame_id = "/map"

        for p_i in range(self.NP):
            particle_msg = Pose()
            px = self.XP[:, p_i]

            q = transformations.quaternion_from_euler(0, 0, px[2])

            particle_msg.position.x = px[0]
            particle_msg.position.y = px[1]
            particle_msg.position.z = 0.0

            particle_msg.orientation.x = q[0]
            particle_msg.orientation.y = q[1]
            particle_msg.orientation.z = q[2]
            particle_msg.orientation.w = q[3]

            particles_msg.poses.append(particle_msg)

        self.pose_pub.publish(particles_msg)

        # pose_cov_pf.covariance = P_3d.reshape([-1]).tolist()
        # self.odom_msg.pose = pose_cov_pf

        # self.pose_pub.publish(self.odom_msg)


if __name__ == "__main__":

    rospy.init_node("PF_estimator", anonymous=True)

    freq = 10

    rate = rospy.Rate(freq)

    loc_sensor = Converter()

    pf = ParticleFilter(dt=1 / freq)

    cmd_gen = CMDPublisher(1 / freq)

    while not rospy.is_shutdown():

        if loc_sensor.x is not None and loc_sensor.y is not None:

            # decide the u
            u = cmd_gen.calc_u()

            # prediction step
            pf.prediction(u)

            # measurement locations
            z = np.array([loc_sensor.x, loc_sensor.y]).reshape([-1, 1])

            # correction step
            pf.correction(z)

            # get the estimated states
            pf.send_estimated_state()

        else:
            pass

        rate.sleep()