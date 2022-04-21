#include <ros/ros.h>
#include "ros/publisher.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/TwistStamped.h"

#include "geometry/odometry.hpp"
#include "proj_const/constants.hpp"


//set values
double clean[] = {0,0,0,0};
//input
Matrix angular_input(4,1);
Matrix tick_input(4,1);
Matrix old_tick_input(4,1,clean);
//output
Matrix angular_speed(3,1);
Matrix tick_speed(3,1);
//calculators
SpeedCalculator angular_calc(Matrix(3,4,round_min_dir_kin),CONTINUE);
SpeedCalculator tick_calc(Matrix(3,4,dis_dir_kin),DISCRETE);
//topics
ros::Publisher angular_vel;
ros::Publisher tick_vel;

void bagMoveCallBack(const sensor_msgs::JointState::ConstPtr& msg){
    //filling out with bag data
    angular_input.fill(msg->velocity.data());
    tick_input.fill(msg->position.data());
    //calculating velocity
    angular_speed = angular_calc << angular_input;
    tick_speed = tick_calc.setTimeStamp(msg->header.stamp.toSec())<< (tick_input - old_tick_input);
    //generating angular message
    geometry_msgs::TwistStamped angular_vel_msg;
    angular_vel_msg.twist.linear.x = angular_speed.get(0, 0);
    angular_vel_msg.twist.linear.y = angular_speed.get(1,0);
    angular_vel_msg.twist.linear.z = 0;
    angular_vel_msg.twist.angular.x = 0;
    angular_vel_msg.twist.angular.y = 0;
    angular_vel_msg.twist.angular.z = angular_speed.get(2,0);
    angular_vel_msg.header.stamp = msg->header.stamp;
    angular_vel_msg.header.frame_id = msg->header.frame_id;
    angular_vel_msg.header.seq = msg->header.seq;
    //generating tick message
    geometry_msgs::TwistStamped tick_vel_msg;
    tick_vel_msg.twist.linear.x = tick_speed.get(0,0);
    tick_vel_msg.twist.linear.y = tick_speed.get(1,0);
    tick_vel_msg.twist.linear.z = 0;
    tick_vel_msg.twist.angular.x = 0;
    tick_vel_msg.twist.angular.y = 0;
    tick_vel_msg.twist.angular.z = tick_speed.get(2,0);
    // replicating header from /wheel_state topic to cmd_vel and angular_cmd_vel
    tick_vel_msg.header.stamp = msg->header.stamp;
    tick_vel_msg.header.frame_id = msg->header.frame_id;
    tick_vel_msg.header.seq = msg->header.seq;

    //pushing messeges
    angular_vel.publish(angular_vel_msg);
    tick_vel.publish(tick_vel_msg);

    // update input values
    old_tick_input = tick_input;
}


int main(int argc,char *argv[]){

    ros::init(argc, argv, "velocity_node");
    
    ros::NodeHandle node_Handler;

    ros::Subscriber bag_sub = node_Handler.subscribe("/wheel_states", 1000, bagMoveCallBack);

    angular_vel = node_Handler.advertise<geometry_msgs::TwistStamped>("angular_cmd_vel",1000);
    tick_vel = node_Handler.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);

    ros::spin();

    return 0;
}
