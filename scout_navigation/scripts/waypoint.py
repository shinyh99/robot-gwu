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

from std_srvs.srv import Empty

GoalPlan = [
    # [turn_in_degree, time_in_sleep]
    [-180, 5.0],
    [90, 3.0],
    [90, 3.0],
    [90, 3.0],
    [-90, 3.0],
]


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
        self.acting = False

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

        # self.goal_reached_action(0)
        # self.goal_reached_action(1)
        # self.goal_reached_action(2)
        # self.goal_reached_action(3)
        # self.goal_reached_action(4)

        self.movebase_client()

    def clear_costmaps(self):
        rospy.wait_for_service("/move_base/clear_costmaps")
        rospy.loginfo("Clear costmaps")

        try:
            clear_costmap = rospy.ServiceProxy(
                "/move_base/clear_costmaps", Empty
            )
            clear_costmap()
        except rospy.ServiceException as e:
            rospy.logwarn("clear_costmap call failed: %s" % e)

    def request_nomotion_update(self):
        rospy.wait_for_service("/request_nomotion_update")
        rospy.loginfo("amcl update")

        try:
            rospy.ServiceProxy("/request_nomotion_update", Empty)
        except rospy.ServiceException as e:
            rospy.logwarn("request_nomotion_update call failed: %s" % e)

    def odom2euler(self, odom):
        (_, _, yaw) = transformations.euler_from_quaternion(
            self.odom2quat(odom)
        )
        return yaw

    def odom2quat(self, odom, negate=False):
        if negate:
            return (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                -odom.pose.pose.orientation.w,
            )
        else:
            return (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            )

    def turn(self, rotation):
        """rotate desired amount

        Args:
            rotation (float): positive: left, negative: right in degree
        """
        self.acting = True
        rospy.loginfo("Turn %0.2f degree started", rotation)

        turned = False
        rate = rospy.Rate(20)

        rad = rotation / 180 * pi
        if self.odom_on:

            quat_original = self.odom2quat(self.odom)
            quat_rotation = transformations.quaternion_from_euler(0, 0, rad)
            quat_new = transformations.quaternion_multiply(
                quat_rotation, quat_original
            )

            while not turned:

                quat_now_inv = self.odom2quat(self.odom, True)
                quat_diff = transformations.quaternion_multiply(
                    quat_new, quat_now_inv
                )
                (
                    _,
                    _,
                    heading_diff,
                ) = transformations.euler_from_quaternion(quat_diff)

                # rospy.loginfo(
                #     "heading_diff: %f",
                #     heading_diff * 180 / pi,
                # )
                if abs(heading_diff) >= 7.5 * pi / 180:
                    self.cmd_vel = Twist()
                    if rad >= 0:
                        self.cmd_vel.angular.z = 0.5

                    else:
                        self.cmd_vel.angular.z = -0.5

                else:
                    self.cmd_vel.angular.z = 0.0
                    turned = True
                    rospy.loginfo("Turn %0.2f degree finished", rotation)

                self.pub_cmd_vel.publish(self.cmd_vel)

                rate.sleep()

        self.acting = False

    def desired_heading(self, rad, change):
        heading_desired = rad + change
        if heading_desired < -pi:
            heading_desired += 2 * pi
        elif heading_desired > pi:
            heading_desired -= 2 * pi
        else:
            pass
        return heading_desired

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_on = True

    def sleep(self, time):
        self.acting = True
        rospy.loginfo("sleep for " + str(time) + " seconds")
        rospy.sleep(time)
        self.acting = False

    def goal_reached_action(self, goal_no):
        (degree_to_turn, time_to_sleep) = GoalPlan[goal_no]
        print(degree_to_turn, time_to_sleep)

        self.turn(degree_to_turn)
        self.sleep(time_to_sleep)

        self.request_nomotion_update()
        self.request_nomotion_update()
        self.clear_costmaps()

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

            self.goal_reached_action(self.goal_cnt - 1)

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
        rospy.loginfo("Starting goals achievements ...")
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
        rate = rospy.Rate(0.2)

        pub_pose_array = rospy.Publisher("waypoint", PoseArray, queue_size=1)
        while not rospy.is_shutdown():
            self.clear_costmaps()
            pub_pose_array.publish(self.pose_array)
            rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")