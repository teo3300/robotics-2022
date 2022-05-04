#include <ros/ros.h>

#include "geometry/odometry.hpp"

#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <odom_reset/Odom_Reset.h>

class IntegrationNode {

    const char *name = "integrator_node";

    ros::NodeHandle n;

    ros::Subscriber cmd_velocity;

    // used by reset position service
    ros::Subscriber odom_reset;

    ros::Publisher position_publish;
    nav_msgs::Odometry position_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped position_tf;

    Matrix velocity = Matrix(3,1);
    Matrix position = Matrix(3,1);
    Integrator integrator;

    void loadVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        velocity(0) = msg->twist.linear.x;
        velocity(1) = msg->twist.linear.y;
        velocity(2) = msg->twist.angular.z;
    }

    void setMethod(Method method) {
        ROS_INFO("Node %s: Setting integration method: %s", name, method == EULER ? "Euler" : "Runge-Kutta");
        integrator.setMethod(method);
    }

public:
    IntegrationNode() {

        // setup topic comunications
        cmd_velocity = n.subscribe("cmd_vel", 1000, &IntegrationNode::integrationCallBack, this);
        position_publish = n.advertise<nav_msgs::Odometry>("odom",1000);

        // subscribe to position reset
        odom_reset = n.subscribe("odom_reset", 1000, &IntegrationNode::odomResetCallback, this);


        setMethod(RUNGE_KUTTA);
    }

    void main_loop() {
        ros::spin();
    }

    void integrationCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        // load velocity data from cmd_vel topic message into velocity Matrix
        loadVelocity(msg);

        // set next timeStamp for integrator (to determine integration period) and integrate velocity
        integrator.setTimeStamp(ros::Time::now().toSec()) << velocity;
        position = integrator.getPosition();

        // generate odometry and transform message header
        position_tf.header.stamp    = position_msg.header.stamp     = ros::Time::now();
        position_tf.header.seq      = position_msg.header.seq       = msg->header.seq;
        position_tf.header.frame_id = position_msg.header.frame_id  = "odom";
        position_tf.child_frame_id  = position_msg.child_frame_id   = "base_link";

        // set position
        position_tf.transform.translation.x = position_msg.pose.pose.position.x = position(0);
        position_tf.transform.translation.y = position_msg.pose.pose.position.y = position(1);
        position_tf.transform.translation.z = position_msg.pose.pose.position.z = 0;
        
        // set rotation
        tf2::Quaternion q;
        q.setRPY(0, 0, position(2));
        position_tf.transform.rotation = position_msg.pose.pose.orientation = tf2::toMsg(q);

        position_publish.publish(position_msg);
        tf_broadcaster.sendTransform(position_tf);
    }

    void odomResetCallback(const odom_reset::Odom_Reset::ConstPtr& msg) {
        integrator.resetPosition();
    }

};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    integrationNode.main_loop();
    return 0;
}