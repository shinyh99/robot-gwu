#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan, CompressedImage


params_lidar = {
    "X": 0,  # meter
    "Y": 0,
    "Z": 0.5,
    "YAW": 0,  # deg
    "PITCH": 0,
    "ROLL": 0,
}


params_cam = {
    "WIDTH": 320,  # image width
    "HEIGHT": 240,  # image height
    "FOV": 60,  # Field of view
    "X": 0.0,  # meter
    "Y": 0,
    "Z": 1.0,
    "YAW": 0,  # deg
    "PITCH": 0.0,
    "ROLL": 0,
}


def translationMtx(x, y, z):

    M = np.array(
        [
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ]
    )

    return M


def rotationMtx(yaw: float, pitch: float, roll: float) -> np.array:

    R_x = np.array(
        [
            [1, 0, 0, 0],
            [0, math.cos(roll), -math.sin(roll), 0],
            [0, math.sin(roll), math.cos(roll), 0],
            [0, 0, 0, 1],
        ]
    )

    R_y = np.array(
        [
            [math.cos(pitch), 0, math.sin(pitch), 0],
            [0, 1, 0, 0],
            [-math.sin(pitch), 0, math.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )

    R_z = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0, 0],
            [math.sin(yaw), math.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    R = np.matmul(R_x, np.matmul(R_y, R_z))

    return R


def transformMTX_lidar2cam(params_lidar: dict, params_cam: dict):

    # Relative position of lidar w.r.t cam

    lidar_pos = [params_lidar.get(i) for i in (["X", "Y", "Z"])]
    cam_pos = [params_cam.get(i) for i in (["X", "Y", "Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(
        translationMtx(x_rel, y_rel, z_rel),
        rotationMtx(np.deg2rad(-90.0), 0.0, 0.0),
    )
    R_T = np.matmul(R_T, rotationMtx(0, 0.0, np.deg2rad(-90.0)))

    # ratota and translate the coordinate of a lidar
    R_T = np.linalg.inv(R_T)

    return R_T


def project2img_mtx(params_cam: dict):

    # transformation matrix from 3D to 2D
    # focal lengths
    fc_x = params_cam["HEIGHT"] / (
        2 * np.tan(np.deg2rad(params_cam["FOV"] / 2))
    )
    fc_y = params_cam["HEIGHT"] / (
        2 * np.tan(np.deg2rad(params_cam["FOV"] / 2))
    )

    # the center of image
    cx = params_cam["WIDTH"] / 2
    cy = params_cam["HEIGHT"] / 2

    # transformation matrix from 3D to 2D
    R_f = np.array([[fc_x, 0, cx], [0, fc_y, cy]])

    return R_f


class LIDAR2CAMTransform:
    def __init__(self, params_cam: dict, params_lidar: dict):

        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):

        xyz_c = np.matmul(
            np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis=1),
            self.RT.T,
        )

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyz_c = xyz_c.T

        xc, yc, zc = (
            xyz_c[0, :].reshape([1, -1]),
            xyz_c[1, :].reshape([1, -1]),
            xyz_c[2, :].reshape([1, -1]),
        )

        xn, yn = xc / (zc + 0.0001), yc / (zc + 0.0001)

        xyi = np.matmul(
            self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0)
        )

        xyi = xyi[0:2, :].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass

        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0] >= 0, xyi[:, 0] < self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1] >= 0, xyi[:, 1] < self.height), :]

        return xyi


def draw_pts_img(img, xi, yi):

    point_np = img

    # Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255, 0, 0), -1)

    return point_np


class SensorCalib:
    def __init__(self):

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.img_callback
        )

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):

        self.R = np.array(msg.ranges)
        self.intens = np.array(msg.intensities)

        x = self.R * np.cos(np.arange(360) / 360 * 2 * np.pi)
        y = self.R * np.sin(np.arange(360) / 360 * 2 * np.pi)
        z = np.zeros_like(x)

        self.xyz = np.concatenate(
            [x.reshape([-1, 1]), y.reshape([-1, 1]), z.reshape([-1, 1])], axis=1
        )


if __name__ == "__main__":

    rospy.init_node("ex_calib", anonymous=True)

    ex_calib_transform = SensorCalib()

    time.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        xyz_p = ex_calib_transform.xyz[
            np.where(ex_calib_transform.xyz[:, 0] >= 0)
        ]

        print(xyz_p)

        intens_p = ex_calib_transform.intens.reshape([-1, 1])
        intens_p = intens_p[np.where(ex_calib_transform.xyz[:, 0] >= 0)]

        xyz_c = ex_calib_transform.l2c_trans.transform_lidar2cam(xyz_p)

        xy_i = ex_calib_transform.l2c_trans.project_pts2img(xyz_c, crop=True)

        img_l2c = draw_pts_img(
            ex_calib_transform.img,
            xy_i[:, 0].astype(np.int32),
            xy_i[:, 1].astype(np.int32),
        )

        cv2.imshow("Lidar2Cam", img_l2c)
        cv2.waitKey(1)
