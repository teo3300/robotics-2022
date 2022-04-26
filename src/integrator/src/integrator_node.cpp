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

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class IntegrationNode {
    // ROS communications handling
    ros::NodeHandle n;
    ros::Subscriber vel;
    ros::Subscriber initial_pos;
    ros::Publisher pos;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
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

    void dumpPosition(geometry_msgs::Pose& pose) {

        // set Point position
        Matrix position = integrator->getPosition();
        pose.position.x = position(0);
        pose.position.y = position(1);
        pose.position.z = 0.0;

        // set Quaternion rotation
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(0, 0, position(2));
        rotation_quaternion.normalize();
        pose.orientation = tf2::toMsg(rotation_quaternion);

        transformStamped.transform.translation.x= position(0);
        transformStamped.transform.translation.y= position(1);
        transformStamped.transform.translation.z= 0.0;

        transformStamped.transform.rotation.x= rotation_quaternion.x();
        transformStamped.transform.rotation.y= rotation_quaternion.y();
        transformStamped.transform.rotation.z= rotation_quaternion.z();
        transformStamped.transform.rotation.w= rotation_quaternion.w();
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
        output_position.header.frame_id = "world";
        output_position.child_frame_id = msg->header.frame_id;

        // construct transfromStamped
        transformStamped.header.seq = msg->header.seq;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id= msg->header.frame_id;

        // construct position for odom and transformStamped
        dumpPosition(output_position.pose.pose);

        // construct twist
        dumpTwist(output_position.twist.twist);

        // publish position to odom
        pos.publish(output_position);

        // publish position as tf broadcast
        br.sendTransform(transformStamped);
    }

public:
    IntegrationNode() {

        // setup topic comunications
        // TODO: move from "cmd_vel" to "angular_cmd_vel"
        vel = n.subscribe("cmd_vel", 1000, &IntegrationNode::integrationCallBack, this);
        initial_pos = n.subscribe("/robot/pose", 1000, &IntegrationNode::positionCallBack, this);
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

    void positionCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
        // set initial position
        Matrix position(3,1);
        position(0) = msg->pose.position.x;
        position(1) = msg->pose.position.y;

        // set initial orientation
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        q.normalize();
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        position(2) = yaw;

        // dump position to integrator
        integrator->resetPosition(position);

        // shutdown to release resources
        initial_pos.shutdown();
    }

};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    integrationNode.main_loop();
    return 0;
}