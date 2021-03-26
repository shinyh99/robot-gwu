#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Header, Float64MultiArray, ColorRGBA
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray

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


class PointList:
    x = []
    y = []

    def __init__(self, values=None):
        self.x = []  # list of float
        self.y = []  # list of float

    def __len__(self):
        return len(self.x)


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

        self.mk_array = MarkerArray()

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

            mk.pose.position.x = _p[0]
            mk.pose.position.y = _p[1]

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

        def draw_marker(points: PointList, h, color, head=0) -> None:

            # Filtered Laser Scan Data
            for x, y in zip(points.x, points.y):
                self.mk_array.markers.append(set_marker((x, y), h, color, head))

        def filter_inf(input: LaserScan) -> PointList:
            x_points = []
            y_points = []

            # filter invalid range
            angle = input.angle_min
            for r in input.ranges:
                if not (math.isinf(r) or math.isnan(r)):

                    x_points.append(r * math.cos(angle))
                    y_points.append(r * math.sin(angle))

                angle += input.angle_increment
            points = PointList()
            points.x = x_points
            points.y = y_points
            points.num_of_points = len(x_points)
            return points

        def grouping(points: PointList, min_size: int = 3) -> list:
            clusters = list()
            pts = PointList()
            pts.x, pts.y = [points.x[0]], [points.y[0]]

            for i in range(1, points.num_of_points):
                x_before, y_before = points.x[i - 1], points.y[i - 1]

                x_now, y_now = points.x[i], points.y[i]
                # get euclidean distance
                d_i = math.sqrt((x_now - x_before) ** 2 + (y_now - y_before) ** 2)
                r_i = math.sqrt(x_now ** 2 + y_now ** 2)

                # Same cluster
                if d_i < self.d_g + r_i * self.d_p:
                    pts.x.append(x_now)
                    pts.y.append(y_now)

                # New Cluster
                else:
                    # only if the cluster has more points than min_size
                    if len(pts) >= min_size:
                        clusters.append(pts)
                    pts = PointList()
                    pts.x, pts.y = [x_now], [y_now]
            else:
                if len(pts) >= min_size:
                    clusters.append(pts)

            return clusters

        def splitting(clusters: list, level: int = 0, min_size: int = 3) -> list:
            clusters_out = list()

            cluster: PointList
            for cluster in clusters:

                # if cluster has less than three point, it can't be divided
                if len(cluster) < 3:
                    clusters_out.append(cluster)

                else:
                    x0, x1 = cluster.x[0], cluster.x[-1]
                    y0, y1 = cluster.y[0], cluster.y[-1]

                    # a*x + b*y + c = 0
                    # a = (y0 - y1), b = (x1 - x0), c = (x0*y1 - x1*y0)
                    a = y0 - y1
                    b = x1 - x0
                    c = x0 * y1 - x1 * y0

                    # farthest point in cluster
                    max_d_j = 0.0
                    index = 0
                    for i, (x, y) in enumerate(zip(cluster.x, cluster.y)):
                        d_j = abs(a * x + b * y + c) / math.sqrt(a ** 2 + b ** 2)
                        if d_j > max_d_j:
                            index = i
                            max_d_j = d_j

                    d_j = max_d_j
                    r_j = math.sqrt(cluster.x[index] ** 2 + cluster.y[index] ** 2)

                    # split.

                    if d_j > self.d_split + self.d_p * r_j:
                        cluster_left, cluster_right = PointList(), PointList()
                        cluster_left.x, cluster_left.y = (
                            cluster.x[0 : index + 1],
                            cluster.y[0 : index + 1],
                        )
                        cluster_right.x, cluster_right.y = (
                            cluster.x[index:],
                            cluster.y[index:],
                        )
                        split_left = splitting([cluster_left], level + 1)
                        split_right = splitting([cluster_right], level + 1)
                        clusters_out.extend(split_left)
                        clusters_out.extend(split_right)

                    # do not split
                    else:
                        clusters_out.append(cluster)

            return clusters_out

        def callback(input: LaserScan):
            self.mk_array = MarkerArray()
            self.__id = 0

            # filter out infinity and get x, y points
            points = PointList()
            points = filter_inf(input)

            ## Grouping
            clusters = grouping(points)

            ## Splitting
            splits = splitting(clusters, level=0)

            ## Drawing
            # draw raw(without inf)
            draw_marker(points, 0.5, color_raw)

            # Draw clusters
            cluster: PointList
            for cluster in clusters:
                # change the height
                draw_marker(points, 1.0, color_cluster)
                cluster_size = len(cluster)

                cluster_head = PointList()
                cluster_head.x = [cluster.x[cluster_size // 2]]
                cluster_head.y = [cluster.y[cluster_size // 2]]
                draw_marker(
                    cluster_head,
                    1.0,
                    color_cluster_head,
                    cluster_size / 200.0,
                )

            # Draw splits
            # split: PointList
            for split in splits:
                # darw splitted cluster
                draw_marker(split, 1.5, color_split)

                split_size = len(split)

                split_head = PointList()
                split_head.x = [split.x[split_size // 2]]
                split_head.y = [split.y[split_size // 2]]
                draw_marker(
                    split_head,
                    1.5,
                    color_split_head,
                    split_size / 200.0,
                )

        _ = rospy.Subscriber("/scan_rate", LaserScan, callback=callback)
        marker_pub = rospy.Publisher("/marker", MarkerArray, queue_size=3)

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            marker_pub.publish(self.mk_array)
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
