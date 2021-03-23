#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

class FooPub:
    def __init__(self):
        self.foo_pub = rospy.Publisher(
            "/fooMsg",
            String,
            queue_size=5
        )
        self.n = 0

    def sendMsg(self):
        _msg = "Hello World " + str(self.n)
        self.n += 1
        self.foo_pub.publish(_msg)

if __name__ == "__main__":
    rospy.init_node("tuto_pub", anonymous=False)
    fp = FooPub()
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        fp.sendMsg()
        rate.sleep()
