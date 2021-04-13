#!/usr/bin/env python3

import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import tf

"""실습1: 전역 경로 생성
- 센서 데이터(GPS, IMU)를 이용해 로봇의 Global Pose를 알 수 있다.
- 로봇의 위치를 기록핚 점들의 집합은 경로가 된다.
- 경로를 생성해 텍스트파읷로 저장하자.
    - 텍스트 파읷에는 x, y, yaw(rad)을 저장하고, 데이터갂 갂격은 탭(\t)으로 구분하고, 매 순갂 기록핚 경로점 사이 데이터는 줄 바꿈(\n)으로 구분핚다.
- ROS 메시지는 nav_msgs/Path 타입을 사용핚다.
"""


class MakePath:
    def __init__(self) -> None:
        rospy.init_node("make_path", anonymous=True)

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=3)
        self.is_odom = False
        self.path_msg = Path()
        self.path_msg.header.frame_id = "/map"
        self.prev_x = 0
        self.prev_y = 0
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("espm")
        full_path = pkg_path + "/path" + "/test.txt"

        with open(full_path, "w") as self.f:
            while not rospy.is_shutdown():
                rospy.spin()

        # self.f = open(full_path, "w")
        # while not rospy.is_shutdown():
        #     rospy.spin()

        # self.f.close()

    def odom_callback(self, msg: Odometry):
        waypint_pose = PoseStamped()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        yaw = tf.transformations.euler_from_quaternion(odom_quaternion)[2]
        if self.is_odom == True:
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
            if distance > 0.1:
                waypint_pose.pose.position.x = x
                waypint_pose.pose.position.y = y
                waypint_pose.pose.orientation.w = 1
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data = "{0}\t{1}\t{2}\n".format(x, y, yaw)
                self.f.write(data)
                self.prev_x = x
                self.prev_y = y
                print(x, y)

        else:
            self.is_odom = True
            self.prev_x = x
            self.prev_x = y


if __name__ == "__main__":
    try:
        test_track = MakePath()
        test_track.run()
    except rospy.ROSInterruptException:
        pass
