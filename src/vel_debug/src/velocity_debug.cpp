#include "ros/node_handle.h"
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/subscriber.h"

void angular_speed_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    ROS_INFO("Recived a velocity message type angular");
    ROS_INFO("Printing some infos");
    ROS_INFO("Vel x :%f ,\tVel y : %f,\tVel z :%f",msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
    ROS_INFO("Ang_Vel x :%f ,\tAng_Vel y : %f,\tAng_Vel z :%f",msg->twist.angular.x,msg->twist.angular.y,msg->twist.angular.z);
    ROS_INFO("Checking timestam in second : %f",ros::Time::now().toSec());
}

void tick_speed_CallBack(const geometry_msgs::TwistStamped::ConstPtr& msg){
    ROS_INFO("Recived a velocity message type tick");
    ROS_INFO("Printing some infos");
    ROS_INFO("Vel x :%f ,\tVel y : %f,\tVel z :%f",msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
    ROS_INFO("Ang_Vel x :%f ,\tAng_Vel y : %f,\tAng_Vel z :%f",msg->twist.angular.x,msg->twist.angular.y,msg->twist.angular.z);
    ROS_INFO("Checking timestam in second : %f\n",ros::Time::now().toSec());
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"vel_debug");
    ros::NodeHandle n;
    ros::Subscriber sub_ang_vel = n.subscribe("angular_cmd_vel",1000,angular_speed_Callback);
    ros::Subscriber sub_tick_vel = n.subscribe("cmd_vel",1000,tick_speed_CallBack);
    ros::spin();
    return 0;
}