#include <ros/ros.h>
#include "integrator/Odom_Reset.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "reset_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<integrator::Odom_Reset>("reset");

    integrator::Odom_Reset srv;

    if (argc == 4) {
        srv.request.new_x = atof(argv[1]);
        srv.request.new_y = atof(argv[2]);
        srv.request.new_theta = atof(argv[3]);
        srv.request.reset_odom = false;
    } else if (argc == 1) {
        srv.request.reset_odom = true;
    } else {
        ROS_ERROR("usage: reset_client [x y theta]\ndo not specify to reset odometry to current robot position");
        return -1;
    }

    if (!client.call(srv)) {
        ROS_ERROR("call failed");
        return 1;
    }
    return 0;
}