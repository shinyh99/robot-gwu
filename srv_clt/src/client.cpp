#include "ros/ros.h"
#include "srv_clt/service.h"
#include <cstdlib> //이건 뭐지?

int main(int argc, char **argv) {
    ros::init(argc, argv, "client");

    if (argc != 3) {
        ROS_INFO("cmd:\trosrun srv_clt client arg0 arg1");
        ROS_INFO("arg0:\tdouble number\narg1:\tdouble number");
        return 1;
    }
    
    ros::NodeHandle nh;

    ros::ServiceClient clt=nh.serviceClient<srv_clt::service>("server");

    srv_clt::service srv;

    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    if(clt.call(srv)) {
        ROS_INFO("send srv, srv.Request.a and srv.Request.b:\t%ld,\t%ld", (long int)srv.request.a, (long int)srv.request.b);
        ROS_INFO("receive srv, srv.Response.result:\t%ld", (long int)srv.response.result);
    }
    else {
        ROS_ERROR("Failed to call service srv_clt");
        return 1;
    }
    return 0;
    

}