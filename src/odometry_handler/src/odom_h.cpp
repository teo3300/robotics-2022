//ros dep
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

//calculus dep
#include "../../dep/constants/constants.hpp"
#include "../../dep/odometry/odometry.hpp"
#include <iostream>

//debug variables
int msg_number = 0;
//include test
void bagCallback(const sensor_msgs::JointState::ConstPtr& msg){
    //DEBUG
    msg_number++;
    ROS_INFO("Getting info for msg number %d",msg_number);
    for(int i=0;i<msg->name.size();i++){
        ROS_INFO("Value for wheel %s",msg->name[i].c_str());
        ROS_INFO("  position : %f",msg->position[i]);
        ROS_INFO("  velocity : %f",msg->velocity[i]);
    }
    //DEBUG ENDS

}

int main(int argc, char *argv[]){
    //init ros node with name odom
    ros::init(argc,argv,"odom");
    ros::NodeHandle node_handle;

    ros::Subscriber bag_input = node_handle.subscribe("wheel_states",1000,bagCallback);

    ros::spin();
    
}