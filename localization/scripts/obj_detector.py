#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridgeError

from utility import SensorCalib, draw_pts_img


# from utility import SensorCalib, draw_pts_img

# from localization.utils import SensorCalib, draw_pts_img


class OBDetect:
    def __init__(self) -> None:
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.callback
        )

        self.lm_pub = rospy.Publisher(
            "/landmark_local", Float64MultiArray, queue_size=3
        )

        self.lm_msg = Float64MultiArray()

    def callback(self, msg: CompressedImage):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except CvBridgeError as e:
            print(e)

        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    def get_bbox(self):
        # obj[0]: red, obj[1]: yellow
        lower_limit = [np.array([0, 160, 90]), np.array([18, 95, 10])]
        upper_limit = [
            np.array([10, 240, 220]),
            np.array([48, 255, 255]),
        ]

        img_bi_list = []

        rects = []

        for i in range(len(lower_limit)):
            img_bi = cv2.inRange(self.img_hsv, lower_limit[i], upper_limit[i])
            img_bi_list.append(img_bi)

        for i, img_bi in enumerate(img_bi_list):

            contours, _ = cv2.findContours(
                img_bi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if len(contours) != 0:
                bbox_tuple = [cv2.boundingRect(cont) for cont in contours]

                bbox_np = np.array(
                    [
                        [x, y, x + w, y + h, x + int(w / 2)]
                        for x, y, w, h in bbox_tuple
                    ]
                )

                cx_mean = np.mean(bbox_np[:, 4])

                bbox_inst_ID = np.arange(bbox_np.shape[0])[
                    abs(bbox_np[:, 4] - cx_mean) < 80
                ].tolist()

                bbox_inst = bbox_np[bbox_inst_ID, :]

                x1 = np.min(bbox_inst[:, 0])
                y1 = np.min(bbox_inst[:, 1])
                x2 = np.max(bbox_inst[:, 2])
                y2 = np.max(bbox_inst[:, 3])
                rects.append([x1, y1, x2 - x1, y2 - y1, i])

        debug = False
        if debug:
            img_concat = img_bi_list[0] + img_bi_list[1]

            cv2.imshow("Filtering", img_concat)
            cv2.waitKey(1)

        return rects

    def pub_landmark(self, lm_list):
        self.lm_msg.data = lm_list.reshape([-1]).tolist()
        self.lm_pub.publish(self.lm_msg)


if __name__ == "__main__":
    rospy.init_node("ex_calib_estimate_xy", anonymous=True)
    ex_calib_transform = SensorCalib()

    ob_detector = OBDetect()

    time.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        xyz_p = ex_calib_transform.xyz[
            np.where(ex_calib_transform.xyz[:, 0] >= 0)
        ]
        xyz_c = ex_calib_transform.l2c_trans.transform_lidar2cam(xyz_p)
        xy_i = ex_calib_transform.l2c_trans.project_pts2img(xyz_c, False)
        xyii = np.concatenate([xy_i, xyz_p], axis=1)

        xyii = ex_calib_transform.l2c_trans.crop_pts(xyii)

        rects = ob_detector.get_bbox()

        obs_list = []

        if len(rects) != 0:
            for (x, y, w, h, id_o) in rects:
                cx = int(x + w / 2)
                cy = int(y + h / 2)

                xy_o = xyii[
                    np.logical_and(
                        xyii[:, 0] >= cx - 0.5 * w, xyii[:, 0] < cx + 0.5 * w
                    ),
                    :,
                ]
                xy_o = xy_o[
                    np.logical_and(
                        xy_o[:, 1] >= cy - 0.5 * h, xy_o[:, 1] < cy + 0.5 * h
                    ),
                    :,
                ]

                xy_o = np.mean(xy_o[:, 2:], axis=0)[:2]

                if not np.isnan(xy_o[0]) and not np.isnan(xy_o[1]):
                    # print(xy_o)
                    obs_list.append(xy_o.tolist() + [id_o])

                if id_o == 0:
                    cv2.rectangle(
                        ex_calib_transform.img,
                        (x, y),
                        (x + w, y + h),
                        (0, 0, 255),
                        2,
                    )
                else:
                    cv2.rectangle(
                        ex_calib_transform.img,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 255),
                        2,
                    )

        ob_detector.pub_landmark(np.array(obs_list))

        img_l2c = draw_pts_img(
            ex_calib_transform.img,
            xy_i[:, 0].astype(np.int32),
            xy_i[:, 1].astype(np.int32),
        )

        cv2.imshow("Lidar2Cam", img_l2c)
        cv2.waitKey(1)
