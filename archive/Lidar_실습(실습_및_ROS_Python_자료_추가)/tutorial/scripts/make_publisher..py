#! /usr/bin/env python

import rospy
from std_msgs.msg import String

class Foo:
    # 생성자
    def __init__(self):
        rospy.init_node("Node_cc", anonymous=False)
        self.msg_pub = rospy.Publisher(
            "/msg",         # Topic 이름
            String,         # Topic Message 타입
            queue_size=5    # Queue 크기
        )

    def sendMsg(self):
        self.msg_pub.publish("Hello world!")


if __name__ == "__main__":
    # 객체가 생성되면서(인스턴스화) 생성자 실행
    foo = Foo()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        foo.sendMsg()
        rate.sleep()