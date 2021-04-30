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
             [-1, 0],   #Down
             [0, -1]]   #Left

class DataProc:
    def __init__(self):
        self.egoPos = [0, 0]
        self.egoHeading = 0
        self.mapData = OccupancyGrid()

    def GetData(self):
        return (self.egoPos, self.egoHeading, self.mapData)

    def callbackOdom(self, msg):
        self.egoPos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        tmpQuat = msg.pose.pose.orientation
        tmpRot = [tmpQuat.x, tmpQuat.y, tmpQuat.z, tmpQuat.w]
        _, _, self.egoHeading = euler_from_quaternion(tmpRot)

    def callbackMap(self, msg):
        self.mapData = msg

class DijkstraPlan:
    def __init__(self):
        self.mapData = OccupancyGrid()
        self.egoPos = [0, 0]
        self.egoIdx = [0, 0]
        self.egoHeading = 0
        self.mapWidth = 0
        self.mapHeight = 0
        self.distRes = 0
        self.costMap = None
        self.distMap = None

    def DataSet(self, mapData, egoPos, egoHead):
        self.egoPos = egoPos
        self.egoHeading = egoHead

        self.mapData = mapData
        self.mapWidth = mapData.info.width
        self.mapHeight = mapData.info.height
        self.distRes = self.mapData.info.resolution
        mapQuat = [mapData.info.origin.orientation.x, mapData.info.origin.orientation.y, mapData.info.origin.orientation.z, mapData.info.origin.orientation.w]
        _, _, mapHeading = euler_from_quaternion(mapQuat)
        mapHeading -= self.egoHeading
        if (self.mapWidth == 0) or (self.mapHeight == 0) or (self.distRes == 0):
            return False
        
        self.costMap = np.reshape(self.mapData.data, (self.mapWidth, self.mapHeight))
        self.distMap = np.zeros(self.costMap.shape)

    def FindLocalGoal(self):
        lowPos = [0, 0]
        lowDist = 999999999999
        
        tmpLocalPos = [(TargetPos[0] - self.egoPos[0]), (TargetPos[1] - self.egoPos[1])]
        tmpLocalDist = np.linalg.norm(tmpLocalPos, ord=2)

        if tmpLocalDist < self.mapWidth/2:
            lowPos = [round(tmpLocalPos[0] + self.mapWidth/2),
                      round(tmpLocalPos[1] + self.mapHeight/2)]
            lowDist = tmpLocalDist
            return (lowDist, lowPos)
        tmpCornerPos = [[self.mapWidth-1, self.mapHeight],
                        [self.mapWidth-1, 0],
                        [0, self.mapHeight-1],
                        [0, 0]]
        tmpCorner1 = [(self.egoPos[0] - self.mapWidth/2),
                      (self.egoPos[1] - self.mapHeight/2)]
        tmpCorner2 = [(self.egoPos[0] - self.mapWidth/2),
                      (self.egoPos[1] + self.mapHeight/2)]
        tmpCorner3 = [(self.egoPos[0] + self.mapWidth/2),
                      (self.egoPos[1] - self.mapHeight/2)]
        tmpCorner4 = [(self.egoPos[0] + self.mapWidth/2),
                      (self.egoPos[1] + self.mapHeight/2)]
        tmpCorner = [tmpCorner1, tmpCorner2, tmpCorner3, tmpCorner4]

        tmpCornerDist1 = np.linalg.norm([(TargetPos[0]-tmpCorner1[0]), (TargetPos[1]-tmpCorner1[1])], ord=2)
        tmpCornerDist2 = np.linalg.norm([(TargetPos[0]-tmpCorner2[0]), (TargetPos[1]-tmpCorner2[1])], ord=2)
        tmpCornerDist3 = np.linalg.norm([(TargetPos[0]-tmpCorner3[0]), (TargetPos[1]-tmpCorner3[1])], ord=2)
        tmpCornerDist4 = np.linalg.norm([(TargetPos[0]-tmpCorner4[0]), (TargetPos[1]-tmpCorner4[1])], ord=2)
        tmpCornerDist = [tmpCornerDist1, tmpCornerDist2, tmpCornerDist3, tmpCornerDist4, lowDist]

        tmpMinIdx = tmpCornerDist.index(min(tmpCornerDist))
        if tmpMinIdx < 4:
            lowDist = tmpCornerDist[tmpMinIdx]
            lowPos = tmpCornerPos[tmpMinIdx]

        for i in range(1, self.mapWidth-1):
            tmpPos1 = [(self.egoPos[0] + (i * self.distRes - self.mapWidth/2)),
                       (self.egoPos[1] + (- self.mapHeight/2))]
            tmpPos2 = [(self.egoPos[0] + (i * self.distRes - self.mapWidth/2)),
                       (self.egoPos[1] + (self.mapHeight/2))]
            tmpDist1 = np.linalg.norm([(TargetPos[0]-tmpPos1[0]), (TargetPos[1]-tmpPos1[1])], ord=2)
            tmpDist2 = np.linalg.norm([(TargetPos[0]-tmpPos2[0]), (TargetPos[1]-tmpPos2[1])], ord=2)
            if (tmpDist1 < tmpDist2) and (tmpDist1 < lowDist) and (self.costMap[i][0] == 0):
                lowPos = [i, 0]
                lowDist = tmpDist1
            elif (tmpDist2 <= tmpDist1) and (tmpDist2 < lowDist) and (self.costMap[i][self.mapHeight-1] == 0):
                lowPos = [i, self.mapHeight-1]
                lowDist = tmpDist2

        for i in range(1, self.mapHeight-1):
            tmpPos1 = [(self.egoPos[0] + (- self.mapWidth/2)),
                       (self.egoPos[1] + (i * self.distRes - self.mapHeight/2))]
            tmpPos2 = [(self.egoPos[0] + (self.mapWidth/2)),
                       (self.egoPos[1] + (i * self.distRes - self.mapHeight/2))]
            tmpDist1 = np.linalg.norm([(TargetPos[0]-tmpPos1[0]), (TargetPos[1]-tmpPos1[1])], ord=2)
            tmpDist2 = np.linalg.norm([(TargetPos[0]-tmpPos2[0]), (TargetPos[1]-tmpPos2[1])], ord=2)
            if (tmpDist1 < tmpDist2) and (tmpDist1 < lowDist) and (self.costMap[0][i] == 0):
                lowPos = [0, self.mapHeight-i]
                lowDist = tmpDist1
            elif (tmpDist2 <= tmpDist1) and (tmpDist2 < lowDist) and (self.costMap[self.mapWidth-1][i] == 0):
                lowPos = [self.mapWidth-1, self.mapHeight-i]
                lowDist = tmpDist2

        return (lowDist, lowPos)

    def Process(self, gPos):
        fromDist = 0
        sPos = [20, 20]
        frontVertex = [(fromDist, sPos[0], sPos[1])]
        searchedVertex = {}
        if (gPos[0] == 20) and (gPos[1] == 20):
            return (True, [])
        goalChk = False

        while len(frontVertex) != 0:
            currentVertex = frontVertex.pop()
            currValue, currCol, currRow = currentVertex
            if (currCol == gPos[0]) and (currRow == gPos[1]):
                goalChk = True
                break
            for Dir in MovingPos:
                tarPos = [(currCol + Dir[0]), (currRow + Dir[1])]
                outlineCheck = (0 <= tarPos[0] < self.mapWidth) and \
                               (0 <= tarPos[1] < self.mapHeight)
                if outlineCheck:
                    doSearch = (self.distMap[tarPos[0]][tarPos[1]] == 0) and \
                               (self.costMap[tarPos[0]][tarPos[1]] == 0)
                    if doSearch:
                        self.distMap[tarPos[0]][tarPos[1]] = 1
                        tmpVertex = ((currValue+1), tarPos[0], tarPos[1])
                        frontVertex.append(tmpVertex)
                        searchedVertex[currentVertex] = tmpVertex

        if goalChk:
            pathRoute = []
            tmpVert = (0, 20, 20)
            while tmpVert in searchedVertex:
                parentVert = searchedVertex[tmpVert]
                pathRoute.append(parentVert)
                tmpVert = parentVert
                pathRoute.sort()
            pointPath = [self.egoPos]

            for vertItem in pathRoute:
                tmpDist, tmpPosX, tmpPosY = vertItem
                tmpPos = [(self.egoPos[0] + (tmpPosX - 20)),
                          (self.egoPos[1] + (tmpPosY - 20))]
                pointPath.append(tmpPos)
            return (True, pointPath)
        else:
            return (False, [])

if __name__ == "__main__":
    rospy.init_node('scout_dijkstra', anonymous=False)
    dijkPlan = DijkstraPlan()
    dataProcess = DataProc()
    rospy.Subscriber('odom', Odometry, dataProcess.callbackOdom, queue_size=1)
    rospy.Subscriber('FixOccuGrid', OccupancyGrid, dataProcess.callbackMap, queue_size=1)

    pubPath = rospy.Publisher('local_path', Path, queue_size=1)
    rosrate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tmpPos, tmpHead, tmpMap = dataProcess.GetData()
        dijkPlan.DataSet(tmpMap, tmpPos, tmpHead)
        tarDist, tarPos = dijkPlan.FindLocalGoal()
        print(tarDist, tarPos)
        findChk, botPath = dijkPlan.Process(tarPos)
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
                tmpPose.pose.position.x = botPath[i][0]
                tmpPose.pose.position.y = botPath[i][1]
                tmpPose.pose.position.z = 0
                tmpPose.pose.orientation = nullQuat
                localPath.poses.append(tmpPose)
            pubPath.publish(localPath)
        rosrate.sleep()
    sys.exit(0)



