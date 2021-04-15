#!/usr/bin/env python3

import sys
import math

import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ca_msgs.msg import CollisionAvoidance
from scout_msgs.msg import ScoutStatus
from morai_msgs.msg import ObjectInfo, GPSMessage

from typing import *

import numpy as np

MaxLeftDegree = 55.0
MaxRightDegree = -55.0
MarginDist = 1

GainAlpha = 1.3
GainBeta = 1


class ObjInfo(object):
    def __init__(self, pos, heading, size, objType):
        self.posX = pos[0]
        self.posY = pos[1]
        self.heading = heading
        self.sizeX = size[0]
        self.sizeY = size[1]
        self.objType = objType
        self.maxCircleRadius = np.sqrt(max(size) ** 2) / 2 + MarginDist

        self.angleFromEgo = 0.0
        self.distFromEgo = 0.0

        self.marginAngleLeft = 0.0
        self.marginAngleRight = 0.0
        self.marginDist = 0.0


class ScoutCollisionAvoidance:
    def __init__(self):
        self.is_there_obj = False
        self.is_changing = False
        self.len_obj = 0
        self.list_obj = []
        self.pastResult = CollisionAvoidance()

        self.egoPosX = 0
        self.egoPosY = 0
        self.egoHeadAngle = 0
        self.egoHeadQuaternion = []

        self.guidAngleLeft = MaxLeftDegree / 180.0 * math.pi
        self.guidAngleRight = MaxRightDegree / 180.0 * math.pi

    def process(self):
        tmpResultMsg = CollisionAvoidance()
        tmpResultMsg.header.stamp = rospy.Time.now()
        if self.len_obj > 0:
            focusObjs = self.filtering()
            lengthObjs = len(focusObjs)
            if lengthObjs < 1:
                return tmpResultMsg
            findNearObjs = sorted(
                focusObjs, key=lambda x: x.distFromEgo, reverse=False
            )
            maxIdx, gapList = self.calcGap(focusObjs, lengthObjs)
            if maxIdx == 0:
                maxIdx = 1
            elif maxIdx == (len(gapList)):
                return tmpResultMsg
            tmpIdx = maxIdx - 1
            tmpDist1 = np.sqrt(
                focusObjs[tmpIdx].maxCircleRadius ** 2
                + focusObjs[tmpIdx].distFromEgo ** 2
            )
            tmpDist2 = np.sqrt(
                focusObjs[tmpIdx - 1].maxCircleRadius ** 2
                + focusObjs[tmpIdx - 1].distFromEgo ** 2
            )
            sumAngle = (
                focusObjs[tmpIdx].marginAngleLeft
                + focusObjs[tmpIdx - 1].marginAngleRight
            )
            tmpNum = tmpDist1 + tmpDist2 * np.cos(sumAngle)
            tmpDen = np.sqrt(
                tmpDist1 ** 2
                + tmpDist2 ** 2
                + (2.0 * tmpDist1 * tmpDist2 * np.cos(sumAngle))
            )
            angleGapC = np.arccos(tmpNum / tmpDen) - (
                focusObjs[tmpIdx].marginAngleLeft
            )
            tmpResultMsg.do_ca = True
            tmpResultMsg.phi_gap = -angleGapC
            print(-angleGapC * 180 / math.pi)
            tmpResultMsg.ca_distance = findNearObjs[0].distFromEgo
            tmpResultMsg.ca_const_alpha = GainAlpha
            tmpResultMsg.ca_const_beta = GainBeta

        return tmpResultMsg

    # Filtering interest zone
    def filtering(self):
        tmpNpArr = np.array(self.list_obj, dtype=object)
        self.list_obj = []
        tmpFilterList = []
        maxIdx = 0
        for obj in tmpNpArr:
            obj.angleFromEgo = (
                np.arctan2((obj.posY - self.egoPosY), (obj.posX - self.egoPosX))
                - self.egoHeadAngle
            )
            obj.distFromEgo = np.linalg.norm(
                [(obj.posX - self.egoPosX), (obj.posY - self.egoPosY)], ord=2
            )
            tmpFilterList.append(obj.angleFromEgo)
        tmpFilterArr = np.asarray(tmpFilterList)
        tmpMask = np.logical_and(
            (tmpFilterArr < self.guidAngleLeft),
            (tmpFilterArr > self.guidAngleRight),
        )
        # print(tmpMask)
        filteredDatas = tmpNpArr[tmpMask]
        filteredAngles = tmpFilterArr[tmpMask]

        sortedDatas = []
        if len(filteredDatas) > 0:
            # sort CCW
            sortedDatas = sorted(
                filteredDatas, key=lambda x: x.angleFromEgo, reverse=True
            )

        return sortedDatas

    def calcGap(self, objs: List[ObjInfo], length: int):
        angleList = [[0, self.guidAngleLeft]]
        gapList = []
        for i in range(length):
            distDatas = [
                (objs[i].posX - self.egoPosX),
                (objs[i].posY - self.egoPosY),
                (objs[i].maxCircleRadius + MarginDist),
            ]
            tmpSquare = (
                distDatas[0] ** 2 + distDatas[1] ** 2 - distDatas[2] ** 2
            )
            if tmpSquare <= 0:
                tmpSquare = distDatas[0] ** 2 + distDatas[1] ** 2
            distSide = np.sqrt(tmpSquare)
            objs[i].marginDist = distSide
            deltaAngle = np.arctan2(objs[i].maxCircleRadius, distSide)
            tmpLeftAngle = objs[i].angleFromEgo + deltaAngle
            tmpRightAngle = objs[i].angleFromEgo - deltaAngle
            objs[i].marginAngleLeft = tmpLeftAngle
            objs[i].marginAngleRight = tmpRightAngle
            angleList.append([tmpLeftAngle, tmpRightAngle])
        angleList.append([self.guidAngleRight, 0])
        # List of gaps starting from left boundary to right boundary
        for i in range(1, len(angleList)):
            tmpDeltaAngleLeft = angleList[i - 1][1] - angleList[i][0]
            gapList.append(tmpDeltaAngleLeft)
        npGapList = np.asarray(gapList)
        maxIdx = np.argmax(npGapList)
        # print(maxIdx, npGapList[maxIdx] / math.pi * 180)
        return (maxIdx, npGapList)

    def callbackObjInfo(self, datas: ObjectInfo):
        self.len_obj = datas.num_of_objects
        if self.len_obj > 0:
            posX = datas.pose_x
            posY = datas.pose_y
            heading = datas.heading
            sizeX = datas.size_x
            sizeY = datas.size_y
            objType = datas.object_type
            self.list_obj = []
            for i in range(0, self.len_obj):
                self.list_obj.append(
                    ObjInfo(
                        [posX[i], posY[i]],
                        heading[i],
                        [sizeX[i], sizeY[i]],
                        objType[i],
                    )
                )
        else:
            self.list_obj = []

    def callbackImu(self, datas):
        self.egoHeadQuaternion = [
            datas.orientation.x,
            datas.orientation.y,
            datas.orientation.z,
            datas.orientation.w,
        ]
        _, _, self.egoHeadAngle = euler_from_quaternion(self.egoHeadQuaternion)

    def callbackOdom(self, datas):
        self.egoPosX = datas.pose.pose.position.x
        self.egoPosY = datas.pose.pose.position.y


if __name__ == "__main__":
    rospy.init_node("scout_ca", anonymous=False)
    scoutCA = ScoutCollisionAvoidance()
    rospy.Subscriber(
        "Object_topic", ObjectInfo, scoutCA.callbackObjInfo, queue_size=None
    )
    rospy.Subscriber("odom", Odometry, scoutCA.callbackOdom, queue_size=1)
    rospy.Subscriber("imu", Imu, scoutCA.callbackImu, queue_size=1)
    pub_ca = rospy.Publisher("ctrl_collision", CollisionAvoidance, queue_size=1)
    rosrate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tmpCaMsg = scoutCA.process()
        pub_ca.publish(tmpCaMsg)
        rosrate.sleep()
    sys.exit(0)
