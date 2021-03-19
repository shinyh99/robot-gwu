#!/usr/bin/env python
# import roslib

# roslib.load_manifest("laser_assembler")

import enum
import rospy
import math
from sensor_msgs.msg import LaserScan
from my_car.msg import List_int
from my_car.msg import Array_int

# from geometry_msgs.msg import PoseArray

from my_car.msg import List_position

# from my_car.srv import srv_range


# from __future__ import print_function

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

# from laser_assembler.srv import *

# raw_data = LaserScan()   # duck type un-neccesary
# ranges = LaserScan().ranges
CLUSTER = Array_int()
POSITION = List_position()


def collect_raw(input):
    ranges = input.ranges

    # * angle in radians
    angle_min = input.angle_min  # -pi/2, left side of car
    angle_max = input.angle_max  # pi/2, right side of car
    # angle_increment = input.angle_increment

    global CLUSTER
    CLUSTER = cluster(ranges, angle_min, angle_max)


def cluster(ranges, angle_min, angle_max):

    length = len(ranges)

    # angles
    angles = [angle_min + i * (angle_max - angle_min) / length for i in range(length)]
    # print(angles)

    # longitudinal, forward
    array_x = [range_ * math.cos(angle) for range_, angle in zip(ranges, angles)]
    # lateral, right
    array_y = [range_ * math.sin(angle) for range_, angle in zip(ranges, angles)]

    global POSITION
    POSITION.x = array_x
    POSITION.y = array_y

    dist_square = []
    for i in range(length - 1):
        dist_square.append(
            (array_x[i] - array_x[i + 1]) ** 2 + (array_y[i] - array_y[i + 1]) ** 2
        )

    # print(ranges)
    group = [[]]
    ls = List_int()
    group = Array_int()
    group_i = 0
    threshold = 0.1  # if the distance btw the points are bigger than 0.1m, then consider them different object
    for i, length in enumerate(dist_square):
        # if the range value is valid
        if not math.isinf(ranges[i + 1]):
            # different group
            if length > threshold:
                group_i += 1
                if len(ls.elements) > 1:
                    group.lists.append(ls)
                ls = List_int()
            # same group
            else:
                pass
            ls.elements.append(i)
    return group


## subscriber
def run():
    rospy.init_node("cluster", anonymous=True)
    _ = rospy.Subscriber("carsim/laser/scan", LaserScan, collect_raw)
    pub_cluster = rospy.Publisher("cluster", Array_int, queue_size=10)
    pub_position = rospy.Publisher("position", List_position, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    global CLUSTER, POSITION

    while not rospy.is_shutdown():
        # rospy.loginfo("%f" % DISTANCE.front)
        # print(DISTANCE)
        pub_cluster.publish(CLUSTER)
        pub_position.publish(POSITION)
        rate.sleep()

    print("Ready to give range")
    # rospy.spin()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
