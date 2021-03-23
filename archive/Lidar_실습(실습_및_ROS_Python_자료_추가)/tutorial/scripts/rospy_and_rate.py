#! /usr/bin/env python3

#! /usr/bin/env python

import rospy

# spin
rospy.init_node("Node_aa", anonymous=False)
"""
To - Do
"""
rospy.spin()


# rate
rospy.init_node("Node_aa", anonymous=False)
"""
To - Do
"""
# rate는 변수니 자유롭게 바꾸셔도 됩니다.
rate = Rate(10) # 10 Hz

# rospy.is_shutdown()은 core가 실행중이면 True, 아니면 False
# not False == True
while not rospy.is_shutdown():
    """
    To - Do
    """
    rate.sleep()