#!/usr/bin/env python

from pickle import GLOBAL
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from opencv_object_tracking.msg import position_publish
import math

# geometry_msgs/Point[] Position_XYZ
#   float64 x
#   float64 y
#   float64 z
# uint16 center_pixel_x
# uint16 center_pixel_y
# uint8 counter

DIRECTION = Twist()
FLAG = False
COUNT = 0
INITIAL_POSITION = Point()


def callback(input):
    # No object detected
    global FLAG, INITIAL_POSITION, DIRECTION, COUNT

    # initialize DIRECTION
    DIRECTION.linear.x = 0
    DIRECTION.linear.y = 0
    DIRECTION.linear.z = 0

    if input.counter == 0:
        if FLAG is True:
            rospy.loginfo("Object lost")
            COUNT = 0
            FLAG = False
        else:
            pass
            # rospy.loginfo("No object is tracked")
    # Object detected
    else:
        if FLAG is False:
            if COUNT == 0:
                rospy.loginfo("Object Found, Initial Posision will be set in 3 sec")
            elif COUNT == 30:
                INITIAL_POSITION.x = input.Position_XYZ[0].x
                INITIAL_POSITION.y = input.Position_XYZ[0].y
                # INITIAL_POSITION.z = input.Position_XYZ[0].z
                rospy.loginfo(
                    "Initial Posision set x:%f\ty:%f\tz:%f"
                    % (INITIAL_POSITION.x, INITIAL_POSITION.y, INITIAL_POSITION.z)
                )
                FLAG = True
            COUNT += 1  # wait until 3 seconds pass after the first detection
        # Object is being tracked just fine
        else:
            DIRECTION.linear.x = -(input.Position_XYZ[0].x - INITIAL_POSITION.x) * 10
            DIRECTION.linear.y = -(input.Position_XYZ[0].y - INITIAL_POSITION.y) * 10
            # DIRECTION.linear.z = input.Position_XYZ[0].z - INITIAL_POSITION.z

    # DIRECTION = input


def magnitude(direction):
    return math.sqrt(direction.linear.x ** 2 + direction.linear.y ** 2)


def control():
    rospy.init_node("control", anonymous=True)
    _ = rospy.Subscriber("position_object", position_publish, callback)
    pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    global DIRECTION

    while not rospy.is_shutdown():
        if magnitude(DIRECTION) < 10:
            pub.publish(DIRECTION)
        else:
            rospy.loginfo("Invalid control, Too large input")
        rate.sleep()


if __name__ == "__main__":
    try:
        control()
    except rospy.ROSInterruptException:
        pass