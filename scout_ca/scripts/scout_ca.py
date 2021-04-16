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
MarginDist = 0.1

GainAlpha = 1.3
GainBeta = 1


def rad2deg(rad):
    return rad * 180.0 / math.pi


def deg2rad(degree):
    return degree * math.pi / 180.0


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

        self.guidAngleLeft = deg2rad(MaxLeftDegree)
        self.guidAngleRight = deg2rad(MaxRightDegree)

    def process(self):
        tmpResultMsg = CollisionAvoidance()
        tmpResultMsg.header.stamp = rospy.Time.now()

        # if object exists
        if self.len_obj > 0:
            focusObjs = self.filtering()
            lengthObjs = len(focusObjs)
            # if object in interest zone does not exist
            if lengthObjs < 1:
                # return nothing
                return tmpResultMsg

            # filter closest to farthest
            findNearObjs = sorted(
                focusObjs, key=lambda x: x.distFromEgo, reverse=False
            )
            # TODO this section is fucked up so hard
            maxIdx, gapList = self.calcGap(focusObjs, lengthObjs)

            # for gap in gapList:
            #     print(gap * 180 / math.pi, end="  ")

            # print(gapList[maxIdx] * 180 / math.pi, sumAngle * 180 / math.pi)
            # # if the largst gap is at left boundary
            # if maxIdx == 0:
            #     maxIdx = 1
            # # ???
            # elif maxIdx == (len(gapList)):
            #     return tmpResultMsg
            # # if the largest gap is at left, tmpIdx is 0
            # # if the largest gap is not at left, tmpIdx is i-1
            # tmpIdx = maxIdx - 1
            # # get distance

            # if the largest gap is at leftmost
            l_bound = 0.0
            r_bound = 0.0
            if maxIdx == 0:
                l_bound = self.guidAngleLeft
                r_bound = focusObjs[maxIdx].marginAngleLeft
                phi_gap_c = self.guidAngleLeft - gapList[maxIdx] / 2
            # if the largest gap is at rightmost
            elif maxIdx == len(gapList) - 1:
                l_bound = focusObjs[maxIdx - 1].marginAngleRight
                r_bound = self.guidAngleRight
                phi_gap_c = self.guidAngleRight + gapList[maxIdx] / 2
            # if the largest gap is between the objects
            else:
                # Do trigometry
                r_obj_idx = maxIdx
                l_obj_idx = int(maxIdx - 1)
                l_bound = focusObjs[l_obj_idx].marginAngleRight
                r_bound = focusObjs[r_obj_idx].marginAngleLeft
                phi_1, phi_2 = abs(r_bound), abs(l_bound)
                d_1 = np.sqrt(
                    focusObjs[r_obj_idx].maxCircleRadius ** 2
                    + focusObjs[r_obj_idx].distFromEgo ** 2
                )
                d_2 = np.sqrt(
                    focusObjs[l_obj_idx].maxCircleRadius ** 2
                    + focusObjs[l_obj_idx].distFromEgo ** 2
                )
                numerator = d_1 + d_2 * np.cos(phi_1 + phi_2)
                denominator = np.sqrt(
                    d_1 ** 2
                    + d_2 ** 2
                    + (2.0 * d_1 * d_2 * np.cos(phi_1 + phi_2))
                )
                phi_gap_c = np.arccos(numerator / denominator) - phi_1

            tmpResultMsg.do_ca = True
            tmpResultMsg.phi_gap = phi_gap_c
            # print(
            #     "idx:",
            #     maxIdx,
            #     "\tgap:",
            #     int(rad2deg(gapList[maxIdx])),
            #     "\tleft:",
            #     int(rad2deg(l_bound)),
            #     "\tright:",
            #     int(rad2deg(r_bound)),
            #     "\tbtw:",
            #     int(rad2deg(phi_gap_c)),
            # )
            # print(
            #     focusObjs[tmpIdx - 1].marginAngleRight * 180 / math.pi,
            #     angleGapC * 180 / math.pi,
            #     focusObjs[tmpIdx].marginAngleLeft * 180 / math.pi,
            # )
            # tmpResultMsg.ca_distance = next(
            #     (
            #         obj_.distFromEgo
            #         for obj_ in enumerate(findNearObjs)
            #         if obj_.distFromEgo != 0.0
            #     ),
            #     None,
            # )
            tmpResultMsg.ca_distance = next(
                (
                    obj.distFromEgo
                    for obj in findNearObjs
                    if obj.distFromEgo != 0
                ),
                None,
            )

            tmpResultMsg.ca_const_alpha = GainAlpha
            tmpResultMsg.ca_const_beta = GainBeta

        return tmpResultMsg

    # Filtering interest zone
    def filtering(self) -> List[ObjInfo]:
        tmpNpArr = np.array(self.list_obj, dtype=object)
        self.list_obj = []
        tmpFilterList = []
        maxIdx = 0

        obj: ObjInfo
        for obj in tmpNpArr:
            # get angle toward object from ego heading
            # Object on left: Positive, Object on right: Negative
            obj.angleFromEgo = (
                np.arctan2((obj.posY - self.egoPosY), (obj.posX - self.egoPosX))
                - self.egoHeadAngle
            )
            # print(obj.angleFromEgo / math.pi * 180)
            # Get distance from ego to object
            obj.distFromEgo = np.linalg.norm(
                [(obj.posX - self.egoPosX), (obj.posY - self.egoPosY)], ord=2
            )
            # print(obj.distFromEgo)
            tmpFilterList.append(obj.angleFromEgo)
        tmpFilterArr = np.asarray(tmpFilterList)

        # Filter by interest range
        tmpMask = np.logical_and(
            (tmpFilterArr < self.guidAngleLeft),
            (tmpFilterArr > self.guidAngleRight),
        )
        # print(tmpMask)
        filteredDatas = tmpNpArr[tmpMask]
        filteredAngles = tmpFilterArr[tmpMask]

        sortedDatas = []
        # If the object exists
        if len(filteredDatas) > 0:
            # sort CW: left(big) to right(small)
            x: ObjInfo
            sortedDatas = sorted(
                filteredDatas, key=lambda x: x.angleFromEgo, reverse=True
            )

        # print("-" * 20)
        # for data in sortedDatas:
        #     print(data.angleFromEgo * 180 / math.pi)

        return sortedDatas

    def calcGap(self, objs: List[ObjInfo], length: int):
        # Left boundary
        angleList = [[0, self.guidAngleLeft]]
        gapList = []
        # Loop through left most object to right most object
        for i in range(length):
            distDatas = [
                (objs[i].posX - self.egoPosX),
                (objs[i].posY - self.egoPosY),
                (objs[i].maxCircleRadius + MarginDist),
            ]
            # get distance from ego to circumscribed circle
            tmpSquare = (
                distDatas[0] ** 2 + distDatas[1] ** 2 - distDatas[2] ** 2
            )
            # if circumscribed circle is too close, then use center distance
            if tmpSquare <= 0:
                tmpSquare = distDatas[0] ** 2 + distDatas[1] ** 2

            # get distance
            distSide = np.sqrt(tmpSquare)
            objs[i].marginDist = distSide
            deltaAngle = np.arctan2(objs[i].maxCircleRadius, distSide)
            tmpLeftAngle = objs[i].angleFromEgo + deltaAngle
            tmpRightAngle = objs[i].angleFromEgo - deltaAngle
            objs[i].marginAngleLeft = tmpLeftAngle
            objs[i].marginAngleRight = tmpRightAngle
            angleList.append([tmpLeftAngle, tmpRightAngle])
        # right boundary
        angleList.append([self.guidAngleRight, 0])
        # List of gaps starting from left boundary to right boundary
        for i in range(1, len(angleList)):
            # Left object's right side - right object's left side
            tmpDeltaAngleLeft = angleList[i - 1][1] - angleList[i][0]
            gapList.append(tmpDeltaAngleLeft)

        # for gap in gapList:
        #     print(gap * 180 / math.pi, end="  ")
        # print("\n", "-" * 20)

        npGapList = np.asarray(gapList)
        # find the biggest gap
        maxIdx = np.argmax(npGapList)
        # print(maxIdx, npGapList[maxIdx] / math.pi * 180)
        return (maxIdx, npGapList)

    def callbackObjInfo(self, datas: ObjectInfo):
        """Get information on objects within few meters from ego

        Args:
            datas (ObjectInfo)
        """
        self.len_obj = datas.num_of_objects
        if self.len_obj > 0:
            posX = datas.pose_x
            posY = datas.pose_y
            heading = datas.heading
            sizeX = datas.size_x
            sizeY = datas.size_y
            objType = datas.object_type
            self.list_obj = []
            # self.list_obj: list of object
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

    def callbackImu(self, datas: Imu):
        self.egoHeadQuaternion = [
            datas.orientation.x,
            datas.orientation.y,
            datas.orientation.z,
            datas.orientation.w,
        ]
        _, _, self.egoHeadAngle = euler_from_quaternion(self.egoHeadQuaternion)

    def callbackOdom(self, datas: Odometry):
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
