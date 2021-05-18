#!/usr/bin/env python3

import rospy
from math import pi

import actionlib

from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Pose,
    Point,
    Quaternion,
    Twist,
)
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseActionGoal,
    MoveBaseActionFeedback,
    MoveBaseActionResult,
)
from actionlib_msgs.msg import GoalStatusArray

# from tf.transformations import euler_from_quaternion

from tf import transformations

# from tf_conversions import transformations

from nav_msgs.msg import Odometry


class MoveBaseSeq:
    def __init__(self):

        rospy.init_node("waypoint")
        # List of goal points:
        points_seq = rospy.get_param("waypoint/p_seq")
        # List of goal quaternions:
        quats_seq = rospy.get_param("waypoint/quat_seq")

        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        # command velocity
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=3)
        self.cmd_vel = Twist()
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.odom = Odometry()
        self.odom_on = False

        # just for display
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "map"

        n = 3
        # Returns a list of lists [[point1(x,y,z,w)], [point2],...[pointn]]
        points = [points_seq[i : i + n] for i in range(0, len(points_seq), n)]
        m = 4
        # Returns a list of lists [[quat1(x,y,z)], [quat2],...[quatn]]
        quats = [quats_seq[i : i + m] for i in range(0, len(quats_seq), m)]

        for point, quat in zip(points, quats):
            # Exploit n variable to cycle in quat_seq
            pose = Pose(Point(*point), Quaternion(*quat))
            self.pose_seq.append(pose)
            self.pose_array.poses.append(pose)

        # Create action client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def turn(self, degree):
        turned = False
        rad = degree / 180 * pi
        if self.odom_on:
            odom_quaternion = (
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w,
            )
            (_, _, yaw_start) = transformations.euler_from_quaternion(
                odom_quaternion
            )
            print(yaw_start)
            yaw_end_right = yaw_start + rad
            yaw_end_left = yaw_start - rad

            print(yaw_start, yaw_end_left, yaw_end_right)

            while not turned:
                odom_quaternion = (
                    self.odom.pose.pose.orientation.x,
                    self.odom.pose.pose.orientation.y,
                    self.odom.pose.pose.orientation.z,
                    self.odom.pose.pose.orientation.w,
                )
                yaw = transformations.euler_from_quaternion(odom_quaternion)

                rate = rospy.Rate(20)

                if (yaw < yaw_end_left) or (yaw > yaw_end_right):
                    turned = True
                else:
                    self.cmd_vel.angular.z = 0.5
                    self.pub_cmd_vel.publish(self.cmd_vel)
                rate.sleep()

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_on = True

    def sleep(self, time):
        rospy.loginfo("sleep for " + str(time))
        rospy.sleep(time)

    def active_cb(self):
        rospy.loginfo(
            "Goal pose "
            + str(self.goal_cnt + 1)
            + " is now being processed by the Action Server..."
        )

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        # rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo(
            "Feedback for goal pose " + str(self.goal_cnt + 1) + " received"
        )

    def done_cb(self, status, result):
        self.goal_cnt += 1
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        # Canceled(cancelled after execution)
        if status == 2:
            rospy.loginfo(
                "Goal pose "
                + str(self.goal_cnt)
                + " received a cancel request after it started executing, completed execution!"
            )

        # Succeeded
        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached")

            self.sleep(10)
            self.turn(90)

            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo(
                    "Sending goal pose "
                    + str(self.goal_cnt + 1)
                    + " to Action Server"
                )
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(
                    next_goal, self.done_cb, self.active_cb, self.feedback_cb
                )
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        # Aborted
        if status == 4:
            rospy.loginfo(
                "Goal pose "
                + str(self.goal_cnt)
                + " was aborted by the Action Server"
            )
            rospy.signal_shutdown(
                "Goal pose " + str(self.goal_cnt) + " aborted, shutting down!"
            )
            return

        # Rejected
        if status == 5:
            rospy.loginfo(
                "Goal pose "
                + str(self.goal_cnt)
                + " has been rejected by the Action Server"
            )
            rospy.signal_shutdown(
                "Goal pose " + str(self.goal_cnt) + " rejected, shutting down!"
            )
            return

        # Recalled(cancelled before execution)
        if status == 8:
            rospy.loginfo(
                "Goal pose "
                + str(self.goal_cnt)
                + " received a cancel request before it started executing, successfully cancelled!"
            )

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo(
            "Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server"
        )
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(
            goal, self.done_cb, self.active_cb, self.feedback_cb
        )
        rate = rospy.Rate(1)

        pub_pose_array = rospy.Publisher("waypoint", PoseArray, queue_size=1)
        while not rospy.is_shutdown():
            pub_pose_array.publish(self.pose_array)
            rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")