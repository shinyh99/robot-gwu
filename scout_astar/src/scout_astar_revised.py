#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math

import rospy
import tf
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped
from scout_msgs.msg import ScoutStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from scipy.ndimage.interpolation import rotate

TargetPos = [-516.5, -496]
GoalRound = 0.6

MovingPos = [[0, 1],    #Up
             [1, 0],    #Right
             [0, -1],   #Left
             [-1, 0]]   #Down

class AVertex:
    def __init__(self, f, g, xy):
        self.x = xy[0]
        self.y = xy[1]
        self.f = f
        self.g = g

class DataProc:
    def __init__(self):
        self.egoPos = [0, 0]
        self.egoHeading = 0
        self.mapData = None

    def GetData(self):
        return (self.egoPos, self.egoHeading, self.mapData)

    def callbackOdom(self, msg):
        self.egoPos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        tmpQuat = msg.pose.pose.orientation
        tmpRot = [tmpQuat.x, tmpQuat.y, tmpQuat.z, tmpQuat.w]
        _, _, self.egoHeading = euler_from_quaternion(tmpRot)

    def callbackMap(self, msg):
        self.mapData = msg

class AstarPlan:
    def __init__(self):
        self.mapData = OccupancyGrid()
        self.egoPos = [0, 0]
        self.egoIdx = [0, 0]
        self.goalIdx = [0, 0]
        self.egoHeading = 0
        self.mapWidth = 0
        self.mapHeight = 0
        self.mapOrigin = [0, 0]
        self.distRes = 1
        self.costMap = None
        self.distMap = None

    def DataSet(self, mapData, egoPos, egoHead):
        self.egoPos = egoPos
        self.egoHeading = egoHead

        self.mapData = mapData
        self.mapWidth = mapData.info.width
        self.mapHeight = mapData.info.height
        self.distRes = self.mapData.info.resolution
        self.mapOrigin = [self.mapData.info.origin.position.x,
                          self.mapData.info.origin.position.y]
        if (self.mapWidth == 0) or (self.mapHeight == 0) or (self.distRes == 0):
            return False
        
        self.costMap = np.reshape(self.mapData.data, (self.mapWidth, self.mapHeight))
        self.distMap = np.zeros(self.costMap.shape)

        tmpX = self.egoPos[0] - self.mapOrigin[0] 
        tmpY = self.egoPos[1] - self.mapOrigin[1]
        self.egoIdx = [int(abs(round(tmpX/self.distRes))),
                       int(abs(round(tmpY/self.distRes)))]
        tmpX = TargetPos[0] - self.mapOrigin[0]
        tmpY = TargetPos[1] - self.mapOrigin[1]
        self.goalIdx = [int(abs(round(tmpX/self.distRes))),
                        int(abs(round(tmpY/self.distRes)))]

    def Process(self, gPos):
        fromDist = 0
        fromCost = np.linalg.norm([(TargetPos[0] - self.egoPos[0]), 
                                   (TargetPos[1] - self.egoPos[1])],
                                  ord=2)
        sPos = self.egoIdx
        startVert = AVertex(fromCost, fromDist, sPos)
        frontVertex = [startVert]
        searchedVertex = {}
        if (gPos[0] == sPos[0]) and (gPos[1] == sPos[1]):
            return (True, [])
        goalChk = False

        while len(frontVertex) != 0:
            tmpVertIdx = 0
            tmpVert = frontVertex[tmpVertIdx]
            for index, item in enumerate(frontVertex):
                if item.f < tmpVert.f:
                    tmpVert = item
                    tmpVertIdx = index
            currentVertex = frontVertex.pop(tmpVertIdx)
            currValue = currentVertex.g
            currCol = currentVertex.x
            currRow = currentVertex.y
            if (currCol == gPos[0]) and (currRow == gPos[1]):
                goalChk = True
                break
            for Dir in MovingPos:
                tarPos = [int(currCol + Dir[0]), int(currRow + Dir[1])]
                outlineCheck = (0 <= tarPos[0] < self.mapWidth) and \
                               (0 <= tarPos[1] < self.mapHeight)
                if outlineCheck:
                    doSearch = (self.distMap[tarPos[0], tarPos[1]] == 0) and \
                               (self.costMap[tarPos[0], tarPos[1]] < 30)
                    if doSearch:
                        self.distMap[tarPos[0], tarPos[1]] = self.distRes
                        heuristic_value = np.linalg.norm([(TargetPos[0] - (tarPos[0]*self.distRes + self.mapOrigin[0])), 
                                                          (TargetPos[1] - (tarPos[1]*self.distRes + self.mapOrigin[1]))],
                                                         ord=2)
                        targetValue = currValue + self.distRes + heuristic_value
                        if currentVertex in searchedVertex:
                            tmpCompVert = searchedVertex[currentVertex]
                            if tmpCompVert.f > targetValue:
                                tmpVertex = AVertex(targetValue, currValue+1, tarPos)
                                searchedVertex[currentVertex] = tmpVertex
                            else:
                                continue
                        else:
                            tmpVertex = (AVertex(targetValue, currValue+1, tarPos))
                            searchedVertex[currentVertex] = tmpVertex
            if currentVertex in searchedVertex:
                frontVertex.append(searchedVertex[currentVertex])
        if goalChk:
            pathRoute = []
            tmpSearchVert = startVert
            while tmpSearchVert in searchedVertex:
                parentVert = searchedVertex[tmpSearchVert]
                pathRoute.append(parentVert)
                tmpSearchVert = parentVert
                pathRoute.sort(key=lambda x : x.g, reverse=True)
            pointPath = []
            
            for vertItem in pathRoute:
                tmpPos = [vertItem.x*0.5 + self.mapOrigin[0],
                          vertItem.y*0.5 + self.mapOrigin[1]]
                pointPath.append(tmpPos)
            return (True, pointPath)
        else:
            return (False, [])

if __name__ == "__main__":
    rospy.init_node('scout_astar', anonymous=False)
    astarPlan = AstarPlan()
    dataProcess = DataProc()
    rospy.Subscriber('odom', Odometry, dataProcess.callbackOdom, queue_size=1)
    rospy.Subscriber('/map', OccupancyGrid, dataProcess.callbackMap, queue_size=1)

    pubPath = rospy.Publisher('local_path', Path, queue_size=1)
    rosrate = rospy.Rate(20)
    print('scout_astar start!')
    while not rospy.is_shutdown():
        tmpPos, tmpHead, tmpMap = dataProcess.GetData()
        if tmpMap is not None: 
            astarPlan.DataSet(tmpMap, tmpPos, tmpHead)
            tmpCoordinate = [(astarPlan.goalIdx[0]*0.5 + astarPlan.mapOrigin[0]),
                             (astarPlan.goalIdx[1]*0.5 + astarPlan.mapOrigin[1])]
            findChk, botPath = astarPlan.Process(astarPlan.goalIdx)
            if findChk:
                localPath = Path()
                localPath.header.frame_id='map'
                nullQuat = Quaternion()
                nullQuat.x = 0
                nullQuat.y = 0
                nullQuat.z = 0
                nullQuat.w = 1
                for i in range(0, len(botPath)):
                    tmpPose = PoseStamped()
                    tmpPath = botPath[i]
                    tmpPose.pose.position.x = tmpPath[0]
                    tmpPose.pose.position.y = tmpPath[1]
                    tmpPose.pose.position.z = 0
                    tmpPose.pose.orientation = nullQuat
                    localPath.poses.append(tmpPose)
                pubPath.publish(localPath)
        rosrate.sleep()
    sys.exit(0)


