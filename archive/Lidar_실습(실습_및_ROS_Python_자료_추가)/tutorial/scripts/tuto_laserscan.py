#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


class LaserProcessor:
    def __init__(self):
        self.ls_pub = rospy.Publisher(
            "/scan",
            LaserScan,
            queue_size=5
        )

    # 무지개빛 총공격이다.
    def sendLaser(self):
        laserData = LaserScan()
        laserData.header.frame_id = "tutorial"
        
        laserData.angle_min = -3.14
        laserData.angle_max = 3.14
        laserData.angle_increment = 6.24 / 999

        laserData.scan_time = 0.1

        laserData.range_min = 0
        laserData.range_max = 10

        laserData.ranges = [i * 0.01 for i in range(0, 999)]

        laserData.intensities = [i for i in range(0, 999)]

        self.ls_pub.publish(laserData)



if __name__ == "__main__":
    rospy.init_node("tuto_laser", anonymous=False)
    lp = LaserProcessor()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        lp.sendLaser()
        rate.sleep()
    