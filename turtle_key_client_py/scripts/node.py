#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

DIRECTION = Twist()

def callback(input):
    global DIRECTION
    DIRECTION = input

def usage():
    return "Wrong number of arguments!!!\n%s\n[r g b width off]"%sys.argv[0]

def key_pen(r, g, b, width, off):
    rospy.init_node('key_pen', anonymous=True)
    rospy.wait_for_service('turtle1/set_pen')  # wait for set_pen service

    request = SetPen()

    request.r = r
    request.g = g
    request.b = b
    request.width = width
    request.off = off

    try:
        response = rospy.ServiceProxy('turtle1/set_pen', SetPen)(r, g, b, width, off)
        print("Requesting r:%s, g:%s, b:%s, width:%s, off:%s"%(r, g, b, width, off))
        print("Result: %s"%response) # response not defined

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    finally: # run regardless status of service
        sub = rospy.Subscriber('cmd_vel', Twist, callback)
        pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=100)
        rate = rospy.Rate(10) # 10hz

        global DIRECTION

        while not rospy.is_shutdown():
            pub.publish(DIRECTION)
            rate.sleep()  

if __name__ == '__main__':
    try:
        if len(sys.argv) == 6:
            r = int(sys.argv[1])
            g = int(sys.argv[2])
            b = int(sys.argv[3])
            width = int(sys.argv[4])
            off = int(sys.argv[5])
            key_pen(r, g, b, width, off)
        else:
            print(usage())
            sys.exit(1)
    except rospy.ROSInterruptException:
        pass
