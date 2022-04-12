#include "ros/init.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  //std::cout << "Non sto facendo un cazzo";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ROS_INFO("I heard: stocazzoooooo");
  ros::spin();

  return 0;
}
