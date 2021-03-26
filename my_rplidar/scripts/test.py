#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

laserscan = LaserScan()


def callback(input):
    global laserscan
    laserscan = input


def change_rate():
    rospy.init_node("key", anonymous=True)
    _ = rospy.Subscriber("/scan", LaserScan, callback)
    pub = rospy.Publisher("/scan_rate", LaserScan, queue_size=10)
    rate = rospy.Rate(5)  # 10hz

    global laserscan
    # rospy.spin()

    while not rospy.is_shutdown():
        pub.publish(laserscan)
        rate.sleep()


if __name__ == "__main__":
    try:
        change_rate()
    except rospy.ROSInterruptException:
        pass