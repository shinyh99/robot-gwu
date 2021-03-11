#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import LaserScan
from my_car.srv import srv_range


def index_to_range(index):
    rospy.init_node("index_to_range", anonymous=True)
    rospy.wait_for_service("index2range")

    request = srv_range()

    request.index = index

    try:
        response = rospy.ServiceProxy("index2range", srv_range)(request.index)
        return response

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "Wrong number of arguments!!!\n%s\t[index]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        index = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting range in %s" % index)
    print("Result: %s" % index_to_range(index))
