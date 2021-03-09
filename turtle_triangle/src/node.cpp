#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv) {
	ros::init(argc, argv, "radius");

    int length = 0;
    // radius is not given
    if (argc != 2) {
        ROS_ERROR("Wrong number of arguments!!!");
		ROS_ERROR("%s [length]", argv[0]);
        return 1;
    } else {
        length = atoll(argv[1]);
    }

    float PI = 3.14159265358979323846;

	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);  // publisher 선언

    geometry_msgs::Twist output;
	
	ros::Rate loop_rate(1);

    int count = 0;
	
	while(ros::ok()) {
        if (count % length == 0)  {
            output.linear.x = 0; // RIGHT
            output.linear.y = 0; // UP
            output.linear.z = 0; // SCREEN

            output.angular.x = 0;
            output.angular.y = 0;
            output.angular.z = PI/180*120;
        } else {
            output.linear.x = 1; // RIGHT
            output.linear.y = 0; // UP
            output.linear.z = 0; // SCREEN

            output.angular.x = 0;
            output.angular.y = 0;
            output.angular.z = 0;
        }        
		
		pub.publish(output);
		loop_rate.sleep();	
        ++count;
	}
	return 0;
}

