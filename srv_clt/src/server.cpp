#include "ros/ros.h"
#include "srv_clt/service.h"

bool calculation(srv_clt::service::Request &req, srv_clt::service::Response &res) {
    res.result = req.a + req.b;

    ROS_INFO("request:\tx=%ld\ty=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("response:\t%ld", (long int)res.result);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "server");
    ros::NodeHandle nh;

    ros::ServiceServer svr=nh.advertiseService("server", calculation);

    ROS_INFO("service server ready");
    ros::spin();

    return 0;
}