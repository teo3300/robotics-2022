#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "geometry/integrator.hpp"

class IntegrationNode {
    ros::NodeHandle n;
    ros::Subscriber vel;
    ros::Publisher pos;
    nav_msgs::Odometry output_pos;
    Matrix* velocity;
    Integrator* integrator;

    void loadVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // fetch linear velocities for x and y axis
        (*velocity)(0,0) = msg->twist.linear.x;
        (*velocity)(1,0) = msg->twist.linear.y;
        // fetch angular velocity for z axis
        (*velocity)(2,0) = msg->twist.angular.z;
    }

    void getInitialPos() {
        Matrix position(3,1);
        if(!n.getParam("inital/position/x", position(0,0))) position(0,0) = 0;
        if(!n.getParam("inital/position/y", position(1,0))) position(1,0) = 0;
        if(!n.getParam("inital/position/t", position(2,0))) position(2,0) = 0;
        integrator->resetPosition(position);
    }

    void dumpPosition() {
        // TODO: convert Matrix position to nav_msgs::Odometry position
        pos.publish(output_pos);
    }

public:
    IntegrationNode() {
        ROS_INFO("IntegratioNode handler setup");
        // setup topic comunications
        vel = n.subscribe("cmd_vel", 1000, &IntegrationNode::integrationCallBack, this);
        pos = n.advertise<nav_msgs::Odometry>("odom", 1000);
        // setup working classes
        velocity = new Matrix(3,1);
        integrator = new Integrator(EULER);
    }
    ~IntegrationNode() { delete velocity; delete integrator; ROS_INFO("IntegratioNode handler destroyed"); }

    void main_loop() {
        ROS_INFO("IntegratioNode handler starting main loop");
        ros::spin();
    }

    void integrationCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        ROS_INFO("IntegratioNode handler got message");
        // load velocity data from cmd_vel topic message into velocity Matrix
        loadVelocity(msg);
        
        // set next timeStamp for integrator (to determine integration period)
        integrator->setTimeStamp(msg->header.stamp.toSec());

        // performs actual integration
        (*integrator) << (*velocity);

        // dump position Matrix into odom topic
        dumpPosition();
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    integrationNode.main_loop();
    return 0;
}