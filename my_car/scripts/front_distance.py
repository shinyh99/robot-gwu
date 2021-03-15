#!/usr/bin/env python
# import roslib

# roslib.load_manifest("laser_assembler")

import enum
import rospy
import math
from sensor_msgs.msg import LaserScan
from my_car.msg import Distance

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
distance = Distance()

# minimum road width: 2.75m
WIDTH = 2.75


def collect_raw(input):
    ranges = input.ranges

    # * angle in radians
    angle_min = input.angle_min  # -pi/2, left side of car
    angle_max = input.angle_max  # pi/2, right side of car
    # angle_increment = input.angle_increment

    trig(ranges, angle_min, angle_max)


def trig(ranges, angle_min, angle_max):

    length = len(ranges)

    # angles
    angles = [angle_min + i * (angle_max - angle_min) / length for i in range(length)]
    # print(angles)

    # longitudinal, forward
    array_x = [range_ * math.cos(angle) for range_, angle in zip(ranges, angles)]
    # lateral, right
    array_y = [range_ * math.sin(angle) for range_, angle in zip(ranges, angles)]

    # for i, angle in enumerate(angles):
    #     print(
    #         "angle:%.2f\trange:%.2f\tx:%.2f\ty:%.2f"
    #         % (angle, ranges[i], array_x[i], array_y[i])
    #     )

    with_in_road = [(-WIDTH / 2 < y and y < WIDTH / 2) for y in array_y]

    front = [array_x[i] for i, flag_in in enumerate(with_in_road) if flag_in]
    # print(front)

    global distance
    distance.front = min(front)


## subscriber
def run():
    rospy.init_node("front_distance", anonymous=True)
    _ = rospy.Subscriber("carsim/laser/scan", LaserScan, collect_raw)
    pub = rospy.Publisher("front_distance", Distance, queue_size=1)

    while not rospy.is_shutdown():
        print(distance)
        pub.publish(distance)

    print("Ready to give range")
    rospy.spin()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
