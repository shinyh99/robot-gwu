#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from scout_msgs.msg import ScoutStatus


class CMDParser:
    def __init__(self) -> None:
        self.status_sub = rospy.Subscriber(
            "/scout_status", ScoutStatus, self.status_callback
        )

        self.u = np.zeros((2,))

    def status_callback(self, msg: ScoutStatus):
        self.u[0] = msg.linear_velocity
        self.u[1] = msg.angular_velocity


class CMDPublisher:
    def __init__(self, dt: float) -> None:
        self.dt = dt
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.cmd_msg = Twist()

        self.t = 0.0

    def calc_u(self) -> list:
        # u = [v_linear, v_angular]
        # linear velocity
        self.u[0] = 0.2
        self.cmd_msg.linear.x = self.u[0]

        # angular velocity
        self.u[1] = 0.5 * np.sin(self.t * 2 * np.pi / 5)
        self.cmd_msg.angular.z = self.u[1]

        self.t += self.dt

        self.cmd_pub.publish(self.cmd_msg)

        return self.u
