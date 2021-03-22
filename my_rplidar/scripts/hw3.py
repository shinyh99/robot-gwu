#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

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
        self.threshold_group = 0.05  # 5cm
        self.threshold_portion = 6.14 / 1000  # 6.14 mrad

        def callback():
            pass

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, callback=callback)


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
        rp_lidar.sphere()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
