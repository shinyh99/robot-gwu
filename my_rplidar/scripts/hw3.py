#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Header, Float64MultiArray, ColorRGBA
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy
import numpy as np

# from typing import NamedTuple

## LaserScan
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32 angle_min
# float32 angle_max
# float32 angle_increment
# float32 time_increment
# float32 scan_time
# float32 range_min
# float32 range_max
# float32[] ranges
# float32[] intensities


# class PointList:
#     def __init__(self, values=None):
#         self.points: list
#         self.points = []
#         pass

#     def __len__(self):
#         return len(self.points)


class RPLidar_3:
    def __init__(self):
        rospy.init_node("hw3", anonymous=True)

    def hw3_1(self):
        def callback(_data):
            filtered_range = [
                r for r in _data.ranges if not (math.isinf(r) or math.isnan(r))
            ]
            farthest_distance = max(filtered_range)
            rospy.loginfo("Farthest Point: %f" % farthest_distance)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, callback=callback)

    def hw3_2(self):
        def callback(_data):
            filtered_range = [r for r in _data.ranges if not math.isinf(r)]
            min_range = min(filtered_range)
            if min_range < 0.5:
                rospy.logwarn("Closest Point: %f" % min_range)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, callback=callback)

    def hw3_3(self):
        filtered_range = Float64MultiArray()

        def callback(_data, result):
            result.data = [r for r in _data.ranges if (not math.isinf(r)) and r < 2]

        callback_lambda = lambda data: callback(data, filtered_range)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, callback=callback_lambda)
        self.laser_pub = rospy.Publisher("/scan_c", Float64MultiArray, queue_size=3)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.laser_pub.publish(filtered_range)
            rate.sleep()

    def hw3_4(self):
        self.flipped = LaserScan()

        def callback(_data):
            self.flipped = _data
            self.flipped.ranges = _data.ranges[::-1]

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, callback=callback)
        self.laser_pub = rospy.Publisher("/scan_flipped", LaserScan, queue_size=3)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.laser_pub.publish(self.flipped)
            rate.sleep()

    def hw3_5(self):
        self.data = LaserScan()

        def callback(_data):
            self.data = _data
            self.data.angle_min = -math.pi
            self.data.angle_max = math.pi
            self.data.angle_increment = (
                self.data.angle_max - self.data.angle_min
            ) / 360

            self.data.time_increment = 0.1

            self.data.ranges = [1] * 361

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, callback=callback)
        self.laser_pub = rospy.Publisher("/hw3_5", LaserScan, queue_size=3)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.laser_pub.publish(self.data)
            rate.sleep()

    def hw3_5_1(self):
        data = LaserScan()

        data.header.frame_id = "laser"
        data.range_min = 0
        data.range_max = 2

        data.angle_min = -math.pi
        data.angle_max = math.pi
        data.angle_increment = (data.angle_max - data.angle_min) / 360

        data.time_increment = 0.1

        data.ranges = [1] * 361

        self.laser_pub = rospy.Publisher("/hw3_5_1", LaserScan, queue_size=3)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.laser_pub.publish(data)
            rate.sleep()

    def sphere(self):
        pcl = PointCloud()

        pcl.header = Header()
        pcl.header.stamp = rospy.Time.now()
        pcl.header.frame_id = "laser"

        theta = []
        angle_increment = 2 * math.pi / 100

        angle = 0
        while angle < 2 * math.pi:
            theta.append(angle)
            angle += angle_increment

        angle = 0
        phi = []
        while angle < math.pi:
            phi.append(angle)
            angle += angle_increment

        r = 1

        for i in theta:
            for j in phi:
                pt = Point32()
                pt.x = r * math.sin(j) * math.cos(i)  # x = r * sin(phi) * cos(theta)
                pt.y = r * math.sin(j) * math.sin(i)  # y = r * sin(phi) * sin(theta)
                pt.z = r * math.cos(j)  # z = r * cos(phi)
                pcl.points.append(pt)

        self.laser_pub = rospy.Publisher("/sphere", PointCloud, queue_size=3)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.laser_pub.publish(pcl)
            rate.sleep()

    def clustering(self):
        # User-define value
        # Threshold for group distance
        # smaller: strick
        # default: 5cm, optimized = 10cm
        self.d_g = 0.15  # m

        # Threshold for group proportion
        # smaller: strict
        # default 6.14mrad, 50mrad
        self.d_p = 6.14 / 1000  # rad

        # User-define value
        # Threshold for split distance
        # larger: strick
        # default: 6cm
        self.d_split = 0.06  # m

        color_raw = ColorRGBA()
        color_raw.r = 1
        color_raw.g = 0
        color_raw.b = 0

        color_cluster = ColorRGBA()
        color_cluster.r = 0
        color_cluster.g = 0
        color_cluster.b = 1

        color_cluster_head = ColorRGBA()
        color_cluster_head.r = 0
        color_cluster_head.g = 1
        color_cluster_head.b = 0

        color_split = ColorRGBA()
        color_split.r = 1
        color_split.g = 1
        color_split.b = 1

        color_split_head = ColorRGBA()
        color_split_head.r = 1
        color_split_head.g = 1
        color_split_head.b = 1

        def set_marker(_p, height, color: ColorRGBA, head: float):
            mk = Marker()
            mk.header.frame_id = "laser"
            mk.ns = "position"
            mk.id = self.__id
            mk.lifetime = rospy.Duration.from_sec(0.1)  # 0.1 sec alive

            mk.type = Marker.SPHERE
            mk.action = Marker.ADD

            mk.pose.position.x = _p.x
            mk.pose.position.y = _p.y

            # subject to change
            mk.pose.position.z = height

            mk.pose.orientation.x = 0.0
            mk.pose.orientation.y = 0.0
            mk.pose.orientation.z = 0.0
            mk.pose.orientation.w = 1.0

            if head > 0:
                mk.scale.x = head
                mk.scale.y = head
                mk.scale.z = head
            else:
                mk.scale.x = 0.02
                mk.scale.y = 0.02
                mk.scale.z = 0.02

            # subject to change
            color.a = 1.0
            mk.color = color

            self.__id += 1

            return mk

        def drawing(pts_list: list, height: float, color: ColorRGBA):
            pts: list

            for pts in pts_list:
                draw_pcl(pts, height)

                pts_size = len(pts)
                pts_head = list()
                pts_head = [pts[pts_size // 2]]
                draw_marker(pts_head, height, color, pts_size / 200.0)

        def draw_pcl(pts: list, height: float):
            pt: Point32
            for pt in pts:
                # * IMPORTANT to DEEPCOPY
                new_pt = deepcopy(pt)
                new_pt.z = height
                self.pcl.points.append(new_pt)

        def draw_marker(points: list, h, color, head=0):

            # Filtered Laser Scan Data
            for pt in points:
                self.mk.markers.append(set_marker((pt), h, color, head))

        def filter_raw(input: LaserScan) -> list:
            pts = list()

            # Use real laser data
            # filter invalid range
            angle = input.angle_min
            for r in input.ranges:
                if not (math.isinf(r) or math.isnan(r)):
                    pt = Point32()
                    pt.x = r * math.cos(angle)
                    pt.y = r * math.sin(angle)

                    pts.append(pt)

                angle += input.angle_increment

            # # Use fabricated values
            # pts_x = np.hstack(
            #     (np.arange(0, 1, 0.01), np.arange(-1, 1, 0.01), np.arange(-1, 0, 0.01))
            # )
            # pts_y = np.hstack((np.ones(100), -1 * np.ones(200), np.ones(100)))

            # for x, y in zip(pts_x, pts_y):
            #     pt = Point32()
            #     pt.x = x
            #     pt.y = y
            #     pts.append(pt)

            return pts

        def grouping(points: list, min_size: int = 3) -> list:
            clusters = list()
            temp_pts = list()
            flag = False

            for i, _ in enumerate(points):
                pt_before = points[i - 1]
                pt_now = points[i]

                # get euclidean distance
                d_i = math.sqrt(
                    (pt_now.x - pt_before.x) ** 2 + (pt_now.y - pt_before.y) ** 2
                )
                r_i = math.sqrt(pt_now.x ** 2 + pt_now.y ** 2)

                # Same cluster
                if d_i < self.d_g + r_i * self.d_p:
                    temp_pts.append(pt_now)

                # New Cluster
                else:
                    if flag is False:
                        points.extend(temp_pts)
                        temp_pts = list()
                        flag = True
                    # only if the cluster has more points than min_size
                    if len(temp_pts) >= min_size:
                        clusters.append(temp_pts)
                    temp_pts = list()
                    temp_pts.append(pt_now)
            else:
                if len(temp_pts) >= min_size:
                    clusters.append(temp_pts)

            return clusters

        def splitting(clusters: list, level: int = 0, min_size: int = 3) -> list:
            clusters_out = list()

            pts: list  # list of point32
            for pts in clusters:

                # if cluster has less than three point, it can't be divided
                if len(pts) < 3:
                    clusters_out.append(pts)

                else:
                    x0, x1 = pts[0].x, pts[-1].x
                    y0, y1 = pts[0].y, pts[-1].y

                    # a*x + b*y + c = 0
                    # a = (y0 - y1), b = (x1 - x0), c = (x0*y1 - x1*y0)
                    a = y0 - y1
                    b = x1 - x0
                    c = x0 * y1 - x1 * y0

                    # farthest point in cluster
                    max_d_j = 0.0
                    index = 0

                    for i, pt in enumerate(pts):
                        d_j = abs(a * pt.x + b * pt.y + c) / math.sqrt(a ** 2 + b ** 2)
                        if d_j > max_d_j:
                            index = i
                            max_d_j = d_j

                    d_j = max_d_j
                    r_j = math.sqrt(pts[index].x ** 2 + pts[index].y ** 2)

                    # split.

                    if d_j > self.d_split + self.d_p * r_j:
                        left_pts, right_pts = list(), list()
                        left_pts = pts[0 : index + 1]
                        right_pts = pts[index:]

                        split_left = splitting([left_pts], level + 1)
                        split_right = splitting([right_pts], level + 1)
                        clusters_out.extend(split_left)
                        clusters_out.extend(split_right)

                    # do not split
                    else:
                        clusters_out.append(pts)

            return clusters_out

        def callback(input: LaserScan):
            self.__id = 0

            # filter out infinity and get x, y points
            points: list
            points = filter_raw(input)

            ## Grouping
            groups = grouping(points)

            ## Splitting
            splits = splitting(groups, level=0)

            ## Drawing
            self.pcl = PointCloud()
            self.pcl.header = Header()
            self.pcl.header.stamp = rospy.Time.now()
            self.pcl.header.frame_id = "laser"
            self.mk = MarkerArray()

            # draw raw(without inf)
            draw_marker(points, 0.5, color_raw)

            drawing(groups, 1.0, color_cluster_head)
            drawing(splits, 1.5, color_split_head)

            count_group = 0
            count_split = 0
            for pt in self.pcl.points:
                if pt.z == 1.0:
                    count_group += 1

                else:
                    count_split += 1

        self.pcl = PointCloud()
        self.mk = MarkerArray()

        _ = rospy.Subscriber("/scan", LaserScan, callback=callback)
        pcl_pub = rospy.Publisher("/pcl", PointCloud, queue_size=10)
        mk_pub = rospy.Publisher("/mk", MarkerArray, queue_size=3)

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            pcl_pub.publish(self.pcl)
            mk_pub.publish(self.mk)

            rate.sleep()


if __name__ == "__main__":
    try:
        # Assignment 3
        rp_lidar = RPLidar_3()

        # # 3.1
        # rp_lidar.hw3_1()

        # # 3.2
        # rp_lidar.hw3_2()

        # # 3.3
        # rp_lidar.hw3_3()

        # # 3.4
        # rp_lidar.hw3_4()

        # # 3.5
        # rp_lidar.hw3_5()

        # # 3.5_1
        # rp_lidar.hw3_5_1()

        # sphere
        # rp_lidar.sphere()

        # clustering
        rp_lidar.clustering()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
