// subscriber and client
#include "ros/ros.h"  // ROS 기본헤더
#include "tx_rx/message.h"
#include "tx_rx/service.h"
#include <cstdlib>

int val1;
int val2;
void callback_1(const tx_rx::message::ConstPtr& msg) {
    val1 = msg->data;
}
void callback_2(const tx_rx::message::ConstPtr& msg) {
    val2 = msg->data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "receiver");

	ros::NodeHandle nh;

	ros::Subscriber sub_1=nh.subscribe("three_per_sec", 100, callback_1);
    ros::Subscriber sub_2=nh.subscribe("ten_per_sec", 100, callback_2);

    ros::ServiceClient clt=nh.serviceClient<tx_rx::service>("server");
    tx_rx::service srv;

    ros::Rate loop_rate(0.1); // every 10 sec

    while(ros::ok()) {
        srv.request.val1 = val1;
        srv.request.val2 = val2;        
        if(clt.call(srv)) {
            ROS_INFO("request:\tval1: %ld,\tval2: %ld", (long int)srv.request.val1, (long int)srv.request.val2);
            ROS_INFO("response:\tresult: %ld", (long int)srv.response.result);
        }
        else {
            ROS_ERROR("Failed to call service tx_rx");
            return 1;
        }
        loop_rate.sleep();
        ros::spinOnce();        
    }
	return 0;
}