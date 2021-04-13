#!/usr/bin/env python3

from os import curdir, read
from sys import path
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

"""실습2: 경로 읽어오기 및 지역경로 생성
- 저장한 텍스트 파일을 읽어서 경로 메시지에 담아서 전역경로를 Publish 핚다.
- 전역경로의 경로점 중 로봇과 가장 가까운 경로점을 탐색핚다.
- 가장 가까운 경로점을 시작으로하는 지역경로를 생성해서 메시지로 Publish 핚다.
    - 원하는 지역경로의 길이만큼 경로점을 가져온다.
"""


class PathPubTf:
    def __init__(self) -> None:
        rospy.init_node("path_pub", anonymous=True)

    def run(self):
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.global_path_pub = rospy.Publisher(
            "/global_path", Path, queue_size=3
        )
        self.local_path_pub = rospy.Publisher("local_path", Path, queue_size=3)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = "/map"

        self.is_odom = False
        self.local_path_size = 50

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("espm")
        full_path = pkg_path + "/path" + "/test.txt"

        with open(full_path, "r") as self.f:
            lines = self.f.readlines()
            for line in lines:
                tmp = line.split()
                read_pose = PoseStamped()
                read_pose.pose.position.x = float(tmp[0])
                read_pose.pose.position.y = float(tmp[1])
                read_pose.pose.orientation.w = 1
                self.global_path_msg.poses.append(read_pose)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.is_odom:
                local_path_msg = Path()
                local_path_msg.header.frame_id = "/map"

                x = self.x
                y = self.y
                min_dis = float("inf")
                current_waypoint = -1
                for i, waypoint in enumerate(self.global_path_msg.poses):
                    distance = sqrt(
                        pow(x - waypoint.pose.position.x, 2)
                        + pow(y - waypoint.pose.position.y, 2)
                    )
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

                if current_waypoint != -1:
                    if current_waypoint + self.local_path_size < len(
                        self.global_path_msg.poses
                    ):
                        for num in range(
                            current_waypoint,
                            current_waypoint + self.local_path_size,
                        ):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = (
                                self.global_path_msg.poses[num].pose.position.x
                            )
                            tmp_pose.pose.position.y = (
                                self.global_path_msg.poses[num].pose.position.y
                            )
                            tmp_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(tmp_pose)

                        else:
                            for num in range(
                                current_waypoint,
                                len(self.global_path_msg.poses),
                            ):
                                tmp_pose = PoseStamped()
                                tmp_pose.pose.position.x = (
                                    self.global_path_msg.poses[
                                        num
                                    ].pose.position.x
                                )
                                tmp_pose.pose.position.y = (
                                    self.global_path_msg.poses[
                                        num
                                    ].pose.position.y
                                )
                                tmp_pose.pose.orientation.w = 1
                                local_path_msg.poses.append(tmp_pose)

                    print(x, y)
                    self.global_path_pub.publish(self.global_path_msg)
                    self.local_path_pub.publish(local_path_msg)
                rate.sleep()

    def odom_callback(self, msg: Odometry):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


if __name__ == "__main__":
    try:
        test_track = PathPubTf()
        test_track.run()
    except rospy.ROSInterruptException:
        pass