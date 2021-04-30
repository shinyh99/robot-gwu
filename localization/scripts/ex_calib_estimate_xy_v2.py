#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float64MultiArray

params_lidar = {
    "X": 0, # meter
    "Y": 0,
    "Z": 0.5,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "X": 0., # meter
    "Y": 0,
    "Z":  1.0,
    "YAW": 0, # deg
    "PITCH": 0.0,
    "ROLL": 0
}

def translationMtx(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M


def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R


def transformMTX_lidar2cam(params_lidar, params_cam):

    #Relative position of lidar w.r.t cam
    lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
    cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(translationMtx(x_rel, y_rel, z_rel), rotationMtx(np.deg2rad(-90.), 0., 0.))
    R_T = np.matmul(R_T, rotationMtx(0, 0., np.deg2rad(-90.)))
    
    #rotate and translate the coordinate of a lidar
    R_T = np.linalg.inv(R_T)

    return R_T


def project2img_mtx(params_cam):

    # focal lengths
    fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    #the center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2
    
    #transformation matrix from 3D to 2D
    R_f = np.array([[fc_x,  0,      cx],
                    [0,     fc_y,   cy]])

    return R_f


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis=1), self.RT.T)

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyz_c = xyz_c.T

        xc, yc, zc = xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

    
def draw_pts_img(img, xi, yi):

    point_np = img

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    return point_np


class SensorCalib:
    
    def __init__(self):
    
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):

        self.R = np.array(msg.ranges)
        self.intens = np.array(msg.intensities)

        x = self.R*np.cos(np.arange(360, dtype=np.float32)/360*2*np.pi)
        y = self.R*np.sin(np.arange(360, dtype=np.float32)/360*2*np.pi)
        z = np.zeros_like(x)

        xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)

        self.xyz = xyz[self.R!=10.0, :]



class OBDetect:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        self.lm_pub = rospy.Publisher("/landmark_local", Float64MultiArray, queue_size=3)

        self.lm_msg = Float64MultiArray()

    def callback(self, msg):

        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    def get_bbox(self):

        lower_object = [np.array([0,160,90]), np.array([15,160,140])] #red
        upper_object = [np.array([10,240,220]), np.array([25,240,220])] #yellow

        img_bi_list = []

        rects = []

        for i in range(len(lower_object)):
    
            img_bi = cv2.inRange(self.img_hsv, lower_object[i], upper_object[i])

            img_bi_list.append(img_bi)

        for i, img_bi in enumerate(img_bi_list):

            contours, _ = cv2.findContours(img_bi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours)!=0:
    
                bbox_tuple = [cv2.boundingRect(cont) for cont in contours]

                bbox_np = np.array([[x, y, x+w, y+h, x + int(w/2), w*h] for x, y, w, h in bbox_tuple])

                area_sort = np.argsort(bbox_np[:,-1])[::-1]

                bbox_np = bbox_np[area_sort, :]

                bbox_ID = np.arange(bbox_np.shape[0])

                # bbox_new = []

                while bbox_ID.shape[0]>1:

                    bbox_inst_ID = np.arange(bbox_np.shape[0])[abs(bbox_np[:, 4] - bbox_np[0, 4])<20].tolist()

                    bbox_not_inst_ID = bbox_ID[abs(bbox_np[:, 4] - bbox_np[0, 4])>=20].tolist()

                    bbox_inst = bbox_np[bbox_inst_ID, :]

                    x1 = np.min(bbox_inst[:, 0])

                    y1 = np.min(bbox_inst[:, 1])

                    x2 = np.max(bbox_inst[:, 2])

                    y2 = np.max(bbox_inst[:, 3])

                    if x2-x1>10:

                        rects.append([x1, y1, x2-x1, y2-y1, i])

                    bbox_np = bbox_np[bbox_not_inst_ID, :]

                    bbox_ID = np.arange(bbox_np.shape[0])
        
        return rects

    def pub_landmark(self, lm_list):

        self.lm_msg.data = lm_list.reshape([-1]).tolist()

        self.lm_pub.publish(self.lm_msg)
        


if __name__ == '__main__':
    
    rospy.init_node('ex_calib_estimate_xy', anonymous=True)

    ex_calib_transform = SensorCalib()

    ob_detector = OBDetect()

    time.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
            
        xyz_p = ex_calib_transform.xyz[np.where(ex_calib_transform.xyz[:, 0]>=0)]

        xyz_c = ex_calib_transform.l2c_trans.transform_lidar2cam(xyz_p)

        xy_i = ex_calib_transform.l2c_trans.project_pts2img(xyz_c, False)

        xyii = np.concatenate([xy_i, xyz_p], axis=1)

        xyii = ex_calib_transform.l2c_trans.crop_pts(xyii)

        rects = ob_detector.get_bbox()

        # print(len(rects))

        obs_list = []

        if len(rects)!=0:

            for (x, y, w, h, id_o) in rects:

                cx = int(x + w/2)
                cy = int(y + h/2)
                
                xy_o = xyii[np.logical_and(xyii[:, 0]>=cx-0.5*w, xyii[:, 0]<cx+0.5*w), :]
                xy_o = xy_o[np.logical_and(xy_o[:, 1]>=cy-0.5*h, xy_o[:, 1]<cy+0.5*h), :]

                xy_o = np.mean(xy_o[:, 2:], axis=0)[:2]

                if not np.isnan(xy_o[0]) and not np.isnan(xy_o[1]):

                    obs_list.append(xy_o.tolist()+[id_o])

                cv2.rectangle(ex_calib_transform.img, (x,y),(x+w,y+h),(0,255,255), 2)

            print(obs_list)

        ob_detector.pub_landmark(np.array(obs_list))
                
        img_l2c = draw_pts_img(ex_calib_transform.img, xy_i[:, 0].astype(np.int32),
                                        xy_i[:, 1].astype(np.int32))
                                            
        cv2.imshow("Lidar2Cam", img_l2c)
        cv2.waitKey(1)

