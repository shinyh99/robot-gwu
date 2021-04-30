#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math

import rospy
import tf
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from scout_msgs.msg import ScoutStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from scipy.ndimage.interpolation import rotate

class MapProc:
    def __init__(self):
        self.mapOrigCheck = False
        self.mapOrigin = [0,0]
        self.mapHeading = 0
        self.egoPos = [0,0]
        self.egoRot = Quaternion()
        self.egoHeading = 0

        self.mapWidth = 40
        self.mapHeight = 40

        self.mapData = OccupancyGrid()

    def callbackOccuGrid(self, msg):
        tmpPos = msg.info.origin.position
        tmpRot = msg.info.origin.orientation
        _, _, tmpYaw = euler_from_quaternion([tmpRot.x, tmpRot.y, tmpRot.z, tmpRot.w])
        self.mapHeading = tmpYaw
        self.egoPos = [tmpPos.x, tmpPos.y]
        if not self.mapOrigCheck:
            tmpZPos = [np.fix(tmpPos.x), np.fix(tmpPos.y)]
            self.mapOrigin = tmpZPos
            self.mapOrigCheck = True
        self.mapWidth = msg.info.width
        self.mapHeight = msg.info.height
        mapDataBuff = []
        for i in range(0,40):
            mapDataBuff.append(msg.data[i*40:i*40+39])
        self.mapData = msg

    def callbackImu(self, msg):
        self.egoQuat = [msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w]
        _, _, self.egoHeading = euler_from_quaternion(self.egoQuat)
    
    def callbackOdom(self, msg):
        self.egoPos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.egoRot = msg.pose.pose.orientation

if __name__ == "__main__":
    rospy.init_node('scout_mapper', anonymous=False)
    mapProc = MapProc()
    rospy.Subscriber('OccupancyGrid', OccupancyGrid, mapProc.callbackOccuGrid, queue_size=1)
    rospy.Subscriber('imu', Imu, mapProc.callbackImu, queue_size=1)
    pubFixMap = rospy.Publisher('FixOccuGrid', OccupancyGrid, queue_size=1)

    rosrate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tmpMap = mapProc.mapData
        tmpRes = tmpMap.info.resolution
        if tmpRes == 0:
            tmpRes = 1
        tmpMap.info.origin.position.x -= tmpMap.info.width/(2/tmpRes)
        tmpMap.info.origin.position.y -= tmpMap.info.height/(2/tmpRes)
        
        tmpMapData = np.reshape(tmpMap.data,
                                (tmpMap.info.width, tmpMap.info.height))
        tmpMapDataRot = rotate(tmpMapData,
                               angle=mapProc.egoHeading/math.pi*180 + 180, reshape=False)
        tmpMapDataRot = np.fliplr(tmpMapDataRot)
        tmpMapDataRot = tmpMapDataRot.astype(np.int8)
        tmpMapDataRot = np.clip(tmpMapDataRot, 0, 100)
        tmpMapTot = np.reshape(tmpMapDataRot, (tmpMap.info.width * tmpMap.info.height))
        tmpMap.data = tmpMapTot.tolist()

        tmpMapRot = quaternion_from_euler(0, 0, 0)
        tmpMap.info.origin.orientation.x = tmpMapRot[0]
        tmpMap.info.origin.orientation.y = tmpMapRot[1]
        tmpMap.info.origin.orientation.z = tmpMapRot[2]
        tmpMap.info.origin.orientation.w = tmpMapRot[3]
        tmpMap.header.frame_id = "map"
        try:
            pubFixMap.publish(tmpMap)
        except Exception as e:
            print(e)

        rosrate.sleep()
    sys.exit(0)

