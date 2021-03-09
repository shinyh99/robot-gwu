#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"

geometry_msgs::Twist output;

void callback(const geometry_msgs::Twist::ConstPtr &input) {
	output = *input;
}

int main(int argc, char **argv) {
	//Initiate ROS
	ros::init(argc, argv, "node");

	int r, g, b, width, off;

	if (argc != 6) {
		ROS_ERROR("Wrong number of arguments!!!");
		ROS_ERROR("%s [r g b width off]", argv[0]);
		return 1;
	}

	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("/cmd_vel", 100, callback);
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

	ros::ServiceClient clt=nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

	turtlesim::SetPen setpen;

	setpen.request.r = atoll(argv[1]);
	setpen.request.g = atoll(argv[2]);
	setpen.request.b = atoll(argv[3]);
	setpen.request.width = atoll(argv[4]);
	setpen.request.off = atoll(argv[5]);


	if(clt.call(setpen)) {
		ROS_INFO("Requesting %d, %d, %d, %d, %d", setpen.request.r, setpen.request.g, setpen.request.b, setpen.request.width, setpen.request.off);
		ROS_INFO("Response: %d", setpen.response);
	}
	else {
		ROS_ERROR("Failed to call service test_service");
		return 1;
	}	

	ros::Rate loop_rate(10);

	while(ros::ok()) {		
		pub.publish(output);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}