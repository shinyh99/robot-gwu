// publisher and server
#include "ros/ros.h"  // ROS 기본헤더
#include "tx_rx/message.h"
#include "tx_rx/service.h"

bool sum(tx_rx::service::Request &req, tx_rx::service::Response &res) {
	res.result = req.val1 + req.val2;

	ROS_ERROR("%ld + %ld = %ld", (long int)req.val1, (long int)req.val2, (long int)res.result);

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "transmitter");  // 노드명 초기화
	ros::NodeHandle nh;  // ROS 시스템과 통신을 위한 노드핸들 선언

	ros::Publisher pub_1=nh.advertise<tx_rx::message>("three_per_sec", 100);
	ros::Publisher pub_2=nh.advertise<tx_rx::message>("ten_per_sec", 100);

	ros::ServiceServer svr=nh.advertiseService("server", sum);
	ROS_INFO("service server(sum) ready");

	ros::Rate loop_rate(10);  // 10Hz

	tx_rx::message msg_1;
	tx_rx::message msg_2;
	msg_1.data = 0;
	msg_2.data = 0;
	
	while(ros::ok()) {
		// run every 10 sec
		if (msg_1.data % 10 == 0) {
			pub_2.publish(msg_2);
			ROS_INFO("val2: %d", msg_2.data);
			msg_2.data += 3;
		}

		pub_1.publish(msg_1);
		ROS_INFO("val1: %d", msg_1.data);
		++msg_1.data;

		loop_rate.sleep();
		ros::spinOnce();

	}
	
	return 0;
}