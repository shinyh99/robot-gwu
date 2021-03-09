#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist output;

void callback(const geometry_msgs::Twist::ConstPtr &input) {
	output = *input;
}

int main(int argc, char **argv) {
	//Initiate ROS
	ros::init(argc, argv, "node");  // 노드명 초기화
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("/cmd_vel", 100, callback);
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

	ros::Rate loop_rate(10);

	while(ros::ok()) {		
		pub.publish(output);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}