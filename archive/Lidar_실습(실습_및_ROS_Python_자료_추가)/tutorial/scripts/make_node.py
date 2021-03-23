#! /usr/bin/env python

import rospy

# class 안쓰는 방법

rospy.init_node("Node_aa", anonymous=False)


#===================================================


# class를 사용하는 방법
class Foo:
    # 생성자
    def __init__(self):
        rospy.init_node("Node_cc", anonymous=False)

# 객체가 생성되면서(인스턴스화) 생성자 실행
foo = Foo()