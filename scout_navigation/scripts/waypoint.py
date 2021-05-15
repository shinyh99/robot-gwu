#!/usr/bin/env python

import math

import rospy


from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from move_base_msgs.msg import (
    MoveBaseActionGoal,
    MoveBaseActionFeedback,
    MoveBaseActionResult,
)
from actionlib_msgs.msg import GoalStatusArray

import rospy
import actionlib


WAYPOINTS = PoseArray()

WAYPOINT_1 = Pose()
WAYPOINT_1.position.x = 30
WAYPOINT_1.position.y = 30
WAYPOINT_1.orientation.z = 0.69
WAYPOINT_1.orientation.w = 0.72

WAYPOINT_2 = Pose()
WAYPOINT_2.position.x = 28.63
WAYPOINT_2.position.y = 27.2
WAYPOINT_2.orientation.z = -0.72
WAYPOINT_2.orientation.w = 0.69

WAYPOINT_3 = Pose()
WAYPOINT_3.position.x = 56.6
WAYPOINT_3.position.y = 26.6
WAYPOINT_3.orientation.z = -0.71
WAYPOINT_3.orientation.w = 0.70

WAYPOINT_4 = Pose()
WAYPOINT_4.position.x = 39.3
WAYPOINT_4.position.y = 0.07
WAYPOINT_4.orientation.z = 0.99
WAYPOINT_4.orientation.w = 0.04

WAYPOINT_5 = Pose()
WAYPOINT_5.position.x = 4.03
WAYPOINT_5.position.y = 1.00
WAYPOINT_5.orientation.z = -0.00
WAYPOINT_5.orientation.w = 0.99

WAYPOINTS.poses.append(
    WAYPOINT_1,
    WAYPOINT_2,
    WAYPOINT_3,
    WAYPOINT_4,
    WAYPOINT_5,
)


if __name__ == "__main__":
    rospy.init_node("waypoint_client")
    client = actionlib.SimpleActionClient("do_dishes", DoDishesAction)
    client.wait_for_server()

    goal = DoDishesGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

# if __name__ == "__main__":
#     rospy.init_node("do_dishes_client")
#     client = actionlib.SimpleActionClient("do_dishes", DoDishesAction)
#     client.wait_for_server()

#     goal = DoDishesGoal()
#     # Fill in the goal here
#     client.send_goal(goal)
#     client.wait_for_result(rospy.Duration.from_sec(5.0))


class Waypoint(object):
    def __init__(self):
        pass


if __name__ == "__main__":
    rospy.init_node("waypoint", anonymous=False)
    waypoint = Waypoint()
    rospy.Subscriber(
        "Object_topic", ObjectInfo, scoutCA.callbackObjInfo, queue_size=None
    )
    rospy.Subscriber("odom", Odometry, scoutCA.callbackOdom, queue_size=1)
    rospy.Subscriber("imu", Imu, scoutCA.callbackImu, queue_size=1)
    pub_ca = rospy.Publisher("ctrl_collision", CollisionAvoidance, queue_size=1)
    rosrate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tmpCaMsg = scoutCA.process()
        pub_ca.publish(tmpCaMsg)
        rosrate.sleep()
    sys.exit(0)
