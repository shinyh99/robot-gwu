#!/usr/bin/env python3


import rospy
import tf
from tf import transformations


from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from scout_msgs.msg import ScoutStatus

from math import pi, sin, cos


class ScoutOdom:
    def __init__(self) -> None:
        self.prev_time = rospy.get_rostime()
        self.is_status = False
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        rospy.Subscriber("/scout_status", ScoutStatus, self.status_callback)
        odom_msg = Odometry()
        odom_msg.header.frame_id = "/odom"
        odom_msg.child_frame_id = "/base_link"
        x, y, z = 0.0, 0.0, 0.0
        heading_rad = 0.0
        freq = 30
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            if self.is_status:
                current_time = rospy.get_rostime()
                interval_time = (current_time - self.prev_time).to_sec()
                linear_vel = self.status_msg.linear_velocity
                angular_vel = self.status_msg.angular_velocity

                linear_x_dot = linear_vel * cos(heading_rad)
                linear_y_dot = linear_vel * sin(heading_rad)

                x += linear_x_dot * interval_time
                y += linear_y_dot * interval_time
                heading_rad += angular_vel * interval_time

                q = transformations.quaternion_from_euler(0, 0, heading_rad)
                br = tf.TransformBroadcaster()
                br.sendTransform(
                    (x, y, z), q, current_time, "/base_link", "/odom"
                )
                # q2 = transformations.quaternion_from_euler(0, 0, 0)
                # br.sendTransform((0, 0, 0), q2, current_time, "/odom", "/map")

                # print(linear_vel, angular_vel)

                odom_msg.header.stamp = current_time
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = z

                odom_msg.twist.twist.linear.x = linear_vel
                odom_msg.twist.twist.angular.z = angular_vel
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]

                self.odom_pub.publish(odom_msg)
                self.prev_time = rospy.get_rostime()

            else:
                pass
            rate.sleep()

    def status_callback(self, msg: ScoutStatus):
        self.is_status = True
        self.status_msg = msg


if __name__ == "__main__":
    rospy.init_node("scout_odometry", anonymous=True)
    scout_odom = ScoutOdom()

    rospy.spin()