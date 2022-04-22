#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

#include "geometry/matrix.hpp"
#include "geometry/integrator.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class IntegrationNode {
    // ROS communications handling
    ros::NodeHandle n;
    ros::Subscriber vel;
    ros::Publisher pos;
    
    // Math structure
    Matrix* velocity;
    Integrator* integrator;

    void loadVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // fetch linear velocities for x and y axis
        (*velocity)(0) = msg->twist.linear.x;
        (*velocity)(1) = msg->twist.linear.y;
        // fetch angular velocity for z axis
        (*velocity)(2) = msg->twist.angular.z;
    }

    void getInitialPos() {
        // if setup server is on set position
        Matrix position(3,1);
        if(!n.getParam("inital/position/x", position(0))) position(0) = 0;
        if(!n.getParam("inital/position/y", position(1))) position(1) = 0;
        if(!n.getParam("inital/position/t", position(2))) position(2) = 0;
        integrator->resetPosition(position);
    }

    void dumpPosition(geometry_msgs::Pose& pose) {

        // set Point position
        Matrix position = integrator->getPosition();
        pose.position.x = position(0);
        pose.position.y = position(1);
        pose.position.z = 0;

        // set Quaternion rotation
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(0,0,position(2));
        pose.orientation = tf2::toMsg(rotation_quaternion);
    }

    void dumpTwist(geometry_msgs::Twist& twist) {

        // set linear velocity
        twist.linear.x = (*velocity)(0);
        twist.linear.y = (*velocity)(1);

        // set angular velocity
        twist.angular.z = (*velocity)(2);
    }

    void dumpOdom(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        // define output message
        nav_msgs::Odometry output_position;

        // construct output msg header
        output_position.header.seq = msg->header.seq;
        output_position.header.stamp = msg->header.stamp;
        output_position.header.frame_id = msg->header.frame_id;
        output_position.child_frame_id = msg->header.frame_id;

        // construct twist
        dumpPosition(output_position.pose.pose);
        dumpTwist(output_position.twist.twist);

        pos.publish(output_position);
    }

public:
    IntegrationNode() {

        // setup topic comunications
        // TODO: move from "cmd_vel" to "angular_cmd_vel"
        vel = n.subscribe("cmd_vel", 1000, &IntegrationNode::integrationCallBack, this);
        pos = n.advertise<nav_msgs::Odometry>("odom", 1000);

        // setup working classes
        velocity = new Matrix(3,1);
        integrator = new Integrator(RUNGE_KUTTA);
    }
    ~IntegrationNode() { delete velocity; delete integrator; }

    void main_loop() {
        ros::spin();
    }

    void integrationCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        // load velocity data from cmd_vel topic message into velocity Matrix
        loadVelocity(msg);

        // set next timeStamp for integrator (to determine integration period)
        integrator->setTimeStamp(msg->header.stamp.toSec());

        // performs actual integration
        (*integrator) << (*velocity);

        // dump position Matrix into odom topic
        dumpOdom(msg);

        Matrix position = integrator->getPosition();
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    integrationNode.main_loop();
    return 0;
}