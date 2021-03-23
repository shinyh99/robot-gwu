#! /usr/bin/env python

import rospy
from std_msgs.msg import String

class Foo:
    # 생성자
    def __init__(self):
        rospy.init_node("Node_cc", anonymous=False)
        self.msg_sub = rospy.Subscriber(
            "/msg",         # Topic 이름
            String,         # Topic Message 타입
            callback=self.receive    # callback 함수 지정
        )

    def receive(self, data):
        print(data)


if __name__ == "__main__":
    # 객체가 생성되면서(인스턴스화) 생성자 실행
    foo = Foo()
    rospy.spin()