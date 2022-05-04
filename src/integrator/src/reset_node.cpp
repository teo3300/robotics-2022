#include <ros/ros.h>
#include "integrator/Odom_Reset.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "reset_client");

    if(argc != 4) {
        ROS_INFO("usage: reset x y theta");
        return 1;
    }

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<integrator::Odom_Reset>("reset");

    integrator::Odom_Reset srv;

    srv.request.new_x = atof(argv[1]);
    srv.request.new_y = atof(argv[2]);
    srv.request.new_theta = atof(argv[3]);

    if (!client.call(srv)) {
        ROS_ERROR("call failed");
        return 1;
    }
    return 0;
}