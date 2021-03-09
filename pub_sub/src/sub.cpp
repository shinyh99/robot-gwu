#include "ros/ros.h"  // ROS 기본헤더
#include "pub_sub/message.h"  // 메세지 파일 헤더(빌드될 때 자동 생성 됨)


void msgCallback(const pub_sub::message::ConstPtr& msg){
	ROS_INFO("receive sec=\t%d", msg -> stamp.sec);
	ROS_INFO("receive nsec=\t%d", msg -> stamp.nsec);
	ROS_INFO("receive data=\t%d", msg -> data);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sub");
	ros::NodeHandle nh;
	
	ros::Subscriber sub=nh.subscribe("topic", 100, msgCallback);
	
	ros::spin();
	return 0;
}
