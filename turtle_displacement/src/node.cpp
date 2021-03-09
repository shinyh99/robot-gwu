#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist output;

void callback(const geometry_msgs::Twist::ConstPtr &input) {
	output = *input;
}

int main(int argc, char **argv) {
	//Initiate ROS
	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("/cmd_vel", 100, callback);
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

	ros::Rate loop_rate(100);

    bool flag = false;
    double vel_x = 0;
    double start = ros::Time::now().toSec();
    double travel_time;

	while(ros::ok()) {
        // is stationary
        if(output.linear.x == 0) {
            // was moving
            if(flag) {
                travel_time = ros::Time::now().toSec()-start;
                ROS_INFO("travel time: %f\tx_displacement: %f",travel_time, travel_time*abs(vel_x));
            }
            flag = false;
            start = ros::Time::now().toSec();
        // is moving
        } else {
            vel_x = output.linear.x;
            flag = true;
        }
		pub.publish(output);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}