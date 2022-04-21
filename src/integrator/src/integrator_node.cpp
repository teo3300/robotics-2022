#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include "geometry/integrator.hpp"

class IntegrationNode {
    ros::NodeHandle n;
    ros::Subscriber vel;
    ros::Publisher pos;
    nav_msgs::Odometry output_pos;
    tf::TransformBroadcaster odom_broadcaster;
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

    void dumpPosition(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        //getting the current position
        Matrix position = integrator->getPosition();
        std::cout << "Position:\n" << position << std::endl << std::endl;
        //create a quaterion using angular velocity \theta
        geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(position.get(2,0));
        // creation tf message(see https://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)
        // penso potrebbe essere skippata nel nostro caso ma non ne sono sicuro
        geometry_msgs::TransformStamped odom_tf_position;
        odom_tf_position.header.stamp = msg->header.stamp;
        odom_tf_position.header.frame_id = "odom";
        odom_tf_position.child_frame_id = "base link";
        odom_tf_position.transform.translation.x = position.get(0,0);
        odom_tf_position.transform.translation.y = position.get(1,0);
        odom_tf_position.transform.translation.z = 0;
        odom_tf_position.transform.rotation = odom_quaternion;
        //publishing tf message (internally)
        odom_broadcaster.sendTransform(odom_tf_position);
        //creation of odometry message
        output_pos.header.stamp = msg->header.stamp;
        output_pos.header.frame_id = "odom";
        output_pos.pose.pose.position.x = position.get(0,0);
        output_pos.pose.pose.position.y = position.get(1,0);
        output_pos.pose.pose.position.z = 0;
        output_pos.pose.pose.orientation = odom_quaternion;
        output_pos.child_frame_id = "base link";
        output_pos.twist.twist.linear.x = msg->twist.linear.x;
        output_pos.twist.twist.linear.y = msg->twist.linear.y;
        output_pos.twist.twist.linear.z = msg->twist.linear.z;
        output_pos.twist.twist.angular.x = msg->twist.angular.x;
        output_pos.twist.twist.angular.y = msg->twist.angular.y;
        output_pos.twist.twist.angular.z = msg->twist.angular.z;
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
        std::cout << "Timestamp: " << msg->header.stamp.toSec();
        // load velocity data from cmd_vel topic message into velocity Matrix
        loadVelocity(msg);
        std::cout << "Velocity:\n"<< (*velocity) << std::endl;
        
        // set next timeStamp for integrator (to determine integration period)
        integrator->setTimeStamp(msg->header.stamp.toSec());

        // performs actual integration
        (*integrator) << (*velocity);

        // dump position Matrix into odom topic
        dumpPosition(msg);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    integrationNode.main_loop();
    return 0;
}