#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include"nav_msgs/Odometry.h"

#include "../../geometry/include/geometry/odometry.hpp"

void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("Got a message from integrator on topic odom");
    ROS_INFO("Checking various info from timestamp");
    ROS_INFO("timestamp : %f,\tframe_id : %s,\tchild_frame_id :%s", msg->header.stamp.toSec(),msg->header.frame_id.c_str(),msg->child_frame_id.c_str());
    ROS_INFO("Checking velocity info");
    ROS_INFO("Vx : %lf,\tVy : %lf,\tVz :%lf",msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
    ROS_INFO("Wx : %lf,\tWy : %lf,\tWz :%lf",msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z);
    std::cout << "Printing vel coovariance "<<"\n";
    double* ptr =const_cast<double* >(msg->twist.covariance.data());
    Matrix a(6,6,ptr);
    std::cout<<a<<"\n";
    ROS_INFO("Cheching pose info");
    ROS_INFO("Px : %lf,\tPy : %lf,\tPz :%lf",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    ROS_INFO("Ox : %lf,\tOy : %lf,\tOz :%lf,\tOw :%lf",msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    std::cout << "Printing  pose coovariance"<<"\n";
    ptr =const_cast<double* >(msg->pose.covariance.data());
    a.fill(ptr);
    std::cout<<a<<"\n";
}

int main(int argc,char *argv[]){
    ros::init(argc, argv, "integration_debug_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("odom",1000,odomCallBack);
    ros::spin();
    return 0;
}