#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from my_car.msg import Distance

DIRECTION = Twist()
DISTANCE = Distance()


def key_call_back(input):
    global DIRECTION
    DIRECTION = input


def distance_call_back(distance):
    global DISTANCE
    DISTANCE = distance


def speed_control(front_distance, direction):
    # Far enough
    rospy.loginfo("%f" % front_distance)
    if front_distance > 4:
        pass
    # Need attention
    else:
        # is moving forward
        if direction.linear.x > 0:
            rospy.logwarn("%f Too Close!!!" % front_distance)
            direction.linear.x = 0  # may require damping

        # not moving
        else:
            pass


def key_pub():
    rospy.init_node("key", anonymous=True)
    _ = rospy.Subscriber("cmd_vel", Twist, key_call_back)
    _ = rospy.Subscriber("front_distance", Distance, distance_call_back)
    pub = rospy.Publisher("carsim/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    global DIRECTION, DISTANCE
    # rospy.spin()

    while not rospy.is_shutdown():
        speed_control(DISTANCE.front, DIRECTION)
        pub.publish(DIRECTION)
        rate.sleep()


if __name__ == "__main__":
    try:
        key_pub()
    except rospy.ROSInterruptException:
        pass