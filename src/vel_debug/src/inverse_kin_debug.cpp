#include "geometry_msgs/TwistStamped.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

#include "geometry/odometry.hpp"
#include "proj_const/constants.hpp"
#include "inverse_rpm/Wheels_Rpm.h"

ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Publisher pub;

void wheels_bag_CallBack(const sensor_msgs::JointState::ConstPtr& msg){
    Matrix m(4,1);
    m.fill(msg->velocity.data());
    ROS_INFO("recived msg from bag ");
    ROS_INFO("front left wheel : %lf",m.get(0,0));
    ROS_INFO("front right wheel : %lf",m.get(1,0));
    ROS_INFO("rear left wheel : %lf",m.get(2,0));
    ROS_INFO("rear right wheel : %lf",m.get(3,0));
    inverse_rpm::Wheels_Rpm wheels_form_bag;
    wheels_form_bag.header.stamp = msg->header.stamp;
    wheels_form_bag.header.frame_id = msg->header.frame_id;
    wheels_form_bag.header.seq = msg->header.seq;
    wheels_form_bag.rpm_fl = m.get(0,0);
    wheels_form_bag.rpm_fr = m.get(1,0);
    wheels_form_bag.rpm_rl = m.get(2,0);
    wheels_form_bag.rpm_rr = m.get(3,0);
    pub.publish(wheels_form_bag);
}

void wheels_rpm_CallBack(const inverse_rpm::Wheels_Rpm::ConstPtr& msg){
    ROS_INFO("Wheel fl : %lf",msg->rpm_fl);
    ROS_INFO("Wheel fr : %lf",msg->rpm_fr);
    ROS_INFO("Wheel rl : %lf",msg->rpm_rl);
    ROS_INFO("Wheel rr : %lf",msg->rpm_rr);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv,"inverse_kin_debug");

    ros::NodeHandle node_Handle;

    sub1 = node_Handle.subscribe("wheels_rpm",1000,wheels_rpm_CallBack);
    sub2 = node_Handle.subscribe("wheels_states",1000,wheels_bag_CallBack);
    pub = node_Handle.advertise<inverse_rpm::Wheels_Rpm>("wheels_rpm_plottable",1000);

    ros::spin();

    return 0;
}