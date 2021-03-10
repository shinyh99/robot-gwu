#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

DIRECTION = Twist()

def callback(input):
    global DIRECTION
    DIRECTION = input

def key_pub():
    rospy.init_node('key_pub', anonymous=True)
    sub = rospy.Subscriber('cmd_vel', Twist, callback)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(10) # 10hz

    global DIRECTION

    while not rospy.is_shutdown():
        pub.publish(DIRECTION)
        rate.sleep()

if __name__ == '__main__':
    try:
        key_pub()
    except rospy.ROSInterruptException:
        pass