#! /usr/bin/env python3
# -*- coding: utf-8 -*-


from yaml.error import Mark
from cmd_publisher import CMDParser, CMDPublisher
from std_msgs.msg import Float64MultiArray, ColorRGBA, Header
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, Point32
from tf import transformations
from visualization_msgs.msg import Marker, MarkerArray


class LMParser:
    def __init__(self) -> None:
        self.lm_sub = rospy.Subscriber(
            "/landmark_local", Float64MultiArray, self.lm_callback
        )
        self.lm_measure = None

    def lm_callback(self, msg: Float64MultiArray):
        lm_list = msg.data

        self.lm_measure = lm_list


def pdf_multivariate_gauss(dx, cov) -> np.ndarray:
    part2 = np.exp(-0.5 * np.matmul(np.matmul(dx.T, np.linalg.inv(cov)), dx))

    part1 = 1 / (
        (2 * np.pi) ** (cov.shape[0] / 2) * (np.linalg.det(cov) ** (1 / 2))
    )

    return np.diag(part1 * part2) + 0.00001


class DrawMarker:
    def __init__(self, pts: list) -> None:
        self.mk = MarkerArray()
        self.pts = pts
        self.__id = 0

        red = ColorRGBA()
        red.r = 255

        yellow = ColorRGBA()
        yellow.r = 255
        yellow.g = 255

        self.mk.markers.append(self.set_marker((self.pts[0]), red))
        self.mk.markers.append(self.set_marker((self.pts[1]), yellow))

    def set_marker(self, pt, color: ColorRGBA):
        mk = Marker()
        mk.header.frame_id = "map"
        mk.ns = "position"
        mk.id = self.__id
        mk.lifetime = rospy.Duration.from_sec(1)

        mk.type = Marker.CYLINDER
        mk.action = Marker.ADD

        mk.pose.position.x = pt.x
        mk.pose.position.y = pt.y

        mk.pose.orientation.x = 0.0
        mk.pose.orientation.y = 0.0
        mk.pose.orientation.z = 0.0
        mk.pose.orientation.w = 1.0

        mk.scale.x = 0.5
        mk.scale.y = 0.5
        mk.scale.z = 1

        color.a = 1.0
        mk.color = color

        self.__id += 1

        return mk


class ParticleFilter:
    def __init__(self, dt=0.05, NP=3000) -> None:
        self.T = dt

        self.NP = NP

        self.XP = np.random.randn(3, self.NP) * 20

        self.limit_yaw()

        self.pw = np.ones((NP,)) / NP

        self.q = 0.02

        self.R = np.diag([0.05, 0.05]) / self.T

        # Red, Yellow
        self.lm_list = [[2.0, 0.0], [-2.0, 0.0]]
        red_lm, yellow_lm = Point32(), Point32()
        red_lm.x = 2.0
        red_lm.y = 0.0
        yellow_lm.x = -2.0
        yellow_lm.y = 0.0

        self.draw = DrawMarker([red_lm, yellow_lm])
        self.draw_pub = rospy.Publisher("/landmark", MarkerArray, queue_size=1)

        self.pose_pub = rospy.Publisher("/pose_pf", PoseArray, queue_size=1)

    def prediction(self, u):

        dX_pre = np.zeros((3, self.NP), dtype=float)
        dX_pre[0, :] = u[0] * np.cos(self.XP[2, :])
        dX_pre[1, :] = u[0] * np.sin(self.XP[2, :])
        dX_pre[2, :] = u[1] + self.q * np.random.randn(1, self.NP)

        self.XP += self.T * dX_pre
        self.limit_yaw()

    def limit_yaw(self):
        over_bool = self.XP[2, :] < -np.pi
        self.XP[2, over_bool] = self.XP[2, over_bool] + 2 * np.pi

        under_bool = self.XP[2, :] > np.pi
        self.XP[2, under_bool] = self.XP[2, under_bool] - 2 * np.pi

    def re_sampling(self, w):
        re_sample_ID = np.random.choice(np.arange(self.NP), self.NP, p=w)

        w = np.ones((self.NP,)) / self.NP

        return re_sample_ID, w

    def landmark_measurement_model(self, lm_g):
        cth = np.cos(self.XP[2, :])
        sth = np.sin(self.XP[2, :])

        lm_local = np.zeros((2, self.NP))

        lm_local[0, :] = (
            lm_g[0] * cth
            + lm_g[1] * sth
            - self.XP[0, :] * cth
            - self.XP[1, :] * sth
        )
        lm_local[1, :] = (
            -lm_g[0] * sth
            + lm_g[1] * cth
            - self.XP[0, :] * sth
            - self.XP[1, :] * cth
        )

        return lm_local

    def correction(self, Z):
        if len(Z) != 0:

            Z = np.array(Z).reshape([-1, 3])

            pw = self.pw

            for i in range(Z.shape[0]):
                id_lm = int(Z[i, 2])

                lm_local = self.landmark_measurement_model(self.lm_list[id_lm])
                dz = lm_local - Z[i, :2].reshape([2, -1])

                pdf_z = pdf_multivariate_gauss(dz, self.R)
                pw = pw * pdf_z

            pw = pw / pw.sum()

            self.Xm = np.dot(self.XP, pw).reshape([-1, 1])

            Xcov = np.dot((self.XP - self.Xm), np.diag(pw)).dot(
                (self.XP - self.Xm).T
            )

            re_sample_ID, self.pw = self.re_sampling(pw)

            XP_new = self.XP[:, re_sample_ID]
            self.XP = XP_new + np.diag([0.05, 0.05, 0.01]).dot(
                np.random.randn(3, self.NP)
            )

            self.limit_yaw()

        else:
            self.Xm = self.XP @ self.pw.reshape([-1, 1])

        print(self.Xm)

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
        self.draw_pub.publish(self.draw.mk)

        # pose_cov_pf.covariance = P_3d.reshape([-1]).tolist()
        # self.odom_msg.pose = pose_cov_pf

        # self.pose_pub.publish(self.odom_msg)


if __name__ == "__main__":

    rospy.init_node("PF_localization", anonymous=True)

    freq = 10
    rate = rospy.Rate(freq)

    pf = ParticleFilter(dt=1 / freq, NP=1000)

    cmd_gen = CMDPublisher(dt=1 / freq)
    scout_status = CMDParser()

    lm_parser = LMParser()

    while not rospy.is_shutdown():

        if lm_parser.lm_measure is not None:

            # Get the u
            u = scout_status.u

            # # Calculate the u
            # u = cmd_gen.calc_u()

            # prediction step
            pf.prediction(u)

            # measurement locations
            z = lm_parser.lm_measure

            # correction step
            pf.correction(z)

            # get the estimated states
            pf.send_estimated_state()

        else:
            pass

        rate.sleep()
