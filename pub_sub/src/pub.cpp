#include "ros/ros.h"  // ROS 기본헤더
#include "pub_sub/message.h"  // 메세지 파일 헤더(빌드될 때 자동 생성 됨)
// #include "std_msgs/String.h" //
//#include "sstream" //

int main(int argc, char **argv) {
	ros::init(argc, argv, "pub");  // 노드명 초기화
	ros::NodeHandle nh;  // ROS 시스템과 통신을 위한 노드핸들 선언
	ros::Publisher pub=nh.advertise<pub_sub::message>("topic", 100);  // publisher 선언
	
	ros::Rate loop_rate(10);  // 10Hz
	pub_sub::message msg;
	
	int count = 0;
	
	while(ros::ok()) {
		msg.stamp = ros::Time::now();
		msg.data = count;
		
		ROS_INFO("send sec=\t%d", msg.stamp.sec);
		ROS_INFO("send nsec=\t%d", msg.stamp.nsec);
		ROS_INFO("send data=\t%d", msg.data);	
		
		pub.publish(msg);
		loop_rate.sleep();
		
		++count;
	}
	return 0;
}

