#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from math import pi, pow, sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud

import numpy as np

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
# sys.path.append(os.path.normpath(os.path.join(current_path, 'mgko_alg/')))


from lib.mgeo.class_defs import *

# import lib.mgeo.class_defs.mgeo_planner_map
from dijkstra import Dijkstra
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class globalPathPlanning:
    def __init__(self):
        rospy.init_node("test", anonymous=True)
        self.global_path_pub = rospy.Publisher(
            "global_path", Path, queue_size=1
        )

        # global link and node
        self.link_pub = rospy.Publisher("link", PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher("node", PointCloud, queue_size=1)

        # Odom: TF from gps_imu_parser
        rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback
        )
        rospy.Subscriber("/odom", Odometry, self.current_pose_callback)

        load_path = os.path.normpath(
            os.path.join(current_path, "lib/mgeo_data/kcity")
        )
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        print("# of nodes: ", len(node_set.nodes))
        print("# of links: ", len(link_set.lines))

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.link_msg = self.getAllLinks()
        self.node_msg = self.getAllNode()
        self.global_planner = Dijkstra(self.nodes, self.links)
        self.is_pose = False

        self.car_max_speed = 3.0

        rate = rospy.Rate(1)  # 20hz
        while not rospy.is_shutdown():

            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)

            rate.sleep()

    def current_pose_callback(self, msg: Odometry):
        self.pose_msg = msg
        self.is_pose = True

    def goal_callback(self, msg: PoseStamped):
        if self.is_pose == True:
            x = self.pose_msg.pose.pose.position.x
            y = self.pose_msg.pose.pose.position.y
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            start_min_dis = float("inf")
            goal_min_dis = float("inf")
            # Find the closest node at start and goal
            for node_idx in self.nodes:
                node_pose_x = self.nodes[node_idx].point[0]
                node_pose_y = self.nodes[node_idx].point[1]
                start_dis = sqrt(
                    pow(x - node_pose_x, 2) + pow(y - node_pose_y, 2)
                )
                goal_dis = sqrt(
                    pow(goal_x - node_pose_x, 2) + pow(goal_y - node_pose_y, 2)
                )
                if start_dis < start_min_dis:
                    start_min_dis = start_dis
                    self.start_node_idx = node_idx
                if goal_dis < goal_min_dis:
                    goal_min_dis = goal_dis
                    self.end_node_idx = node_idx

            _, path = self.global_planner.find_shortest_path(
                self.start_node_idx, self.end_node_idx
            )
            dijkstra_path = Path()
            dijkstra_path.header.frame_id = "map"
            for waypoint in path["point_path"]:
                tmp_point = PoseStamped()
                tmp_point.pose.position.x = waypoint[0]
                tmp_point.pose.position.y = waypoint[1]
                dijkstra_path.poses.append(tmp_point)

            self.global_path_pub.publish(dijkstra_path)

            # Calculate curve_based_velocity
            self.curve_based_velocity(dijkstra_path, 0.72)

    def curve_based_velocity(
        self, global_path: Path, road_friction: float = 0.72
    ):
        out_vel_plan = []
        for i in range(self.start_node_idx):
            out_vel_plan.append(self.car_max_speed)

        for i in range(
            self.start_node_idx, len(global_path.poses) - self.start_node_idx
        ):
            a_list = []
            b_list = []
            # loop through 0 to len(global_path.poses)
            for box in range(-self.start_node_idx, self.start_node_idx):
                x = global_path.poses[i + box].pose.position.x
                y = global_path.poses[i + box].pose.position.y
                a_list.append([-2 * x, -2 * y, 1])
                b_list.append(-(x * x) - (y * y))

            a_matrix = np.array(a_list)
            b_matrix = np.array(b_list)
            a_transpose = a_matrix.T

            x_matrix = (
                np.linalg.inv(a_transpose.dot(a_matrix))
                .dot(a_transpose)
                .dot(b_matrix)
            )
            a = x_matrix[0]
            b = x_matrix[1]
            c = x_matrix[2]
            r = sqrt(a * a + b * b - c)

            v_max = sqrt(r * 9.8 * road_friction)
            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(
            len(global_path.poses) - self.start_node_idx, len(global_path.poses)
        ):
            out_vel_plan.append(0)

        return out_vel_plan

    def getAllLinks(self):
        all_link = PointCloud()
        all_link.header.frame_id = "map"

        for link_idx in self.links:
            for link_point in self.links[link_idx].points:
                tmp_point = Point32()
                tmp_point.x = link_point[0]
                tmp_point.y = link_point[1]
                tmp_point.z = 0
                all_link.points.append(tmp_point)

        return all_link

    def getAllNode(self):
        all_node = PointCloud()
        all_node.header.frame_id = "map"
        for node_idx in self.nodes:
            tmp_point = Point32()
            tmp_point.x = self.nodes[node_idx].point[0]
            tmp_point.y = self.nodes[node_idx].point[1]
            tmp_point.z = 0
            all_node.points.append(tmp_point)

        return all_node


if __name__ == "__main__":

    test = globalPathPlanning()
