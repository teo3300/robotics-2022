#include <ros/ros.h>

#include "geometry/odometry.hpp"

#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <integrator/dinIntegratorConfig.h>

class IntegrationNode {

    ros::NodeHandle n;

    ros::Subscriber cmd_velocity;

    ros::Publisher position_publish;
    nav_msgs::Odometry position_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped position_tf;

    dynamic_reconfigure::Server<integrator::dinIntegratorConfig> dynServer;
    dynamic_reconfigure::Server<integrator::dinIntegratorConfig>::CallbackType reconfigureCallBack;

    Matrix velocity = Matrix(3,1);
    Matrix position = Matrix(3,1);
    Integrator integrator;

    void loadVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        velocity(0) = msg->twist.linear.x;
        velocity(1) = msg->twist.linear.y;
        velocity(2) = msg->twist.angular.z;
    }

public:
    IntegrationNode() {

        // setup topic comunications
        cmd_velocity = n.subscribe("cmd_vel", 1000, &IntegrationNode::integrationCallBack, this);
        position_publish = n.advertise<nav_msgs::Odometry>("odom",1000);

        integrator.setMethod(RUNGE_KUTTA);

        //NON SO A COSA SERVA _1
        reconfigureCallBack = boost::bind(&dynamic_Reconfigure_CallBack,&integrator_enum, _1);
        dynServer.setCallback(reconfigureCallBack);
    }

    void main_loop() {
        ros::spin();
    }

    void integrationCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        // load velocity data from cmd_vel topic message into velocity Matrix
        loadVelocity(msg);

        // set next timeStamp for integrator (to determine integration period) and integrate velocity
        integrator.setTimeStamp(msg->header.stamp.toSec()) << velocity;
        position = integrator.getPosition();

        // generate odometry and transform message header
        position_tf.header.stamp    = position_msg.header.stamp     = msg->header.stamp;
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

    void dynamic_Reconfigure_CallBack(char* method,integrator::dinIntegratorConfig &config/*Manca il bitmask level*/){
        //TODO : cose da fare qui quando succede un reconfigure dinamico
    }

};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    integrationNode.main_loop();
    return 0;
}