#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

class FooSub:
    def __init__(self):
        self.something_sub = rospy.Subscriber(
            "/fooMsg",
            String,
            callback=self.callback
        )
        
    def callback(self, data):
        print(data)
    
if __name__ == "__main__":
    rospy.init_node("tuto_sub", anonymous=False)
    fs = FooSub()
    rospy.spin()