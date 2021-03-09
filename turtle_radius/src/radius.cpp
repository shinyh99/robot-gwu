#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "radius");

    int radius = 0;
    // radius is not given
    if (argc != 2) {
        ROS_ERROR("Wrong number of arguments!!!");
		ROS_ERROR("%s [radius]", argv[0]);
        return 1;
    } else {
        radius = atoll(argv[1]);
    }

	ros::NodeHandle nh;  // ROS 시스템과 통신을 위한 노드핸들 선언
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);  // publisher 선언

    geometry_msgs::Twist output;
	
	ros::Rate loop_rate(10);  // 10Hz
	
	while(ros::ok()) {
		// Auto rotation
        output.linear.x = radius; // RIGHT
        output.linear.y = 0; // UP
        output.linear.z = 0; // SCREEN

        output.angular.x = 0;
        output.angular.y = 0;
        output.angular.z = 1;
		
		pub.publish(output);
		loop_rate.sleep();	
	}
	return 0;
}

