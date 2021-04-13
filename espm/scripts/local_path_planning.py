#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from math import pi, pow, sqrt
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Float32MultiArray


class localPathPlanning:
    def __init__(self):
        rospy.init_node("localPathPlanning", anonymous=True)
        self.local_path_pub = rospy.Publisher("local_path", Path, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.current_pose_callback)
        rospy.Subscriber("/global_path", Path, self.path_callback)

        self.local_path_point_num = 50
        self.is_pose = False
        self.is_path = False

        rate = rospy.Rate(30)  # 20hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_pose:

                local_path = Path()
                x = self.pose_msg.pose.pose.position.x
                y = self.pose_msg.pose.pose.position.y
                current_waypoint = -1

                min_dis = float("inf")
                for i in range(len(self.path_msg.poses)):
                    dx = x - self.path_msg.poses[i].pose.position.x
                    dy = y - self.path_msg.poses[i].pose.position.y
                    dis = sqrt(dx * dx + dy * dy)
                    if dis < min_dis:
                        min_dis = dis
                        current_waypoint = i

                if current_waypoint + self.local_path_point_num < len(
                    self.path_msg.poses
                ):
                    end_waypoint = current_waypoint + self.local_path_point_num

                else:
                    end_waypoint = len(self.path_msg.poses)

                local_path.header.frame_id = "map"
                for i in range(current_waypoint, end_waypoint):
                    tmp_pose = PoseStamped()
                    tmp_pose.pose.position.x = self.path_msg.poses[
                        i
                    ].pose.position.x
                    tmp_pose.pose.position.y = self.path_msg.poses[
                        i
                    ].pose.position.y
                    tmp_pose.pose.position.z = self.path_msg.poses[
                        i
                    ].pose.position.z
                    tmp_pose.pose.orientation.x = 0
                    tmp_pose.pose.orientation.y = 0
                    tmp_pose.pose.orientation.z = 0
                    tmp_pose.pose.orientation.w = 1
                    local_path.poses.append(tmp_pose)

                self.local_path_pub.publish(local_path)

            rate.sleep()

    def current_pose_callback(self, msg):
        self.pose_msg = msg
        self.is_pose = True

    def path_callback(self, msg):
        self.path_msg = msg
        self.is_path = True


if __name__ == "__main__":

    test = localPathPlanning()
