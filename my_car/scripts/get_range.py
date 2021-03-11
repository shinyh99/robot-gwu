#!/usr/bin/env python
# import roslib

# roslib.load_manifest("laser_assembler")

import rospy
from sensor_msgs.msg import LaserScan
from my_car.srv import srv_range

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

raw_data = LaserScan()
ranges = LaserScan().ranges


def collect_raw(input):
    global raw_data, ranges
    # raw_data = input
    ranges = input.ranges
    # print(len(ranges))


## server
def get_range(req):
    try:
        range_ = ranges[req.index]
        print("index: %s\trange: %s" % (req.index, range_))
        # print(type(range_))
        return range_
    except:
        print("Invalid index")
        return rospy.ROSException


## subscriber
def run():
    rospy.init_node("get_range", anonymous=True)
    _ = rospy.Subscriber("carsim/laser/scan", LaserScan, collect_raw)

    _ = rospy.Service("index2range", srv_range, get_range)
    print("Ready to give range")
    rospy.spin()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
