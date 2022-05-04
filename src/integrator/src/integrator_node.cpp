#include <ros/ros.h>

#include "geometry/odometry.hpp"

#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <integrator/Odom_Reset.h>

#include <dynamic_reconfigure/server.h>
#include <integrator/dinIntegratorConfig.h>

unsigned long treshold;
unsigned long tTresh;
double cx, cy, ct;

int integrator_enum = 0;

nav_msgs::Odometry position_msg;
nav_msgs::Odometry odometry_msg;

geometry_msgs::TransformStamped odometry_tf;
geometry_msgs::TransformStamped position_tf;

tf2::Quaternion q;

Integrator node_integrator(0);

class IntegrationNode {

    ros::NodeHandle n;

    ros::Subscriber cmd_velocity;
    ros::Subscriber get_odom;

    ros::Publisher position_publish;
    ros::Publisher odometry_publish;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    dynamic_reconfigure::Server<integrator::dinIntegratorConfig> dynServer;

    Matrix velocity = Matrix(3,1);
    Matrix position = Matrix(3,1);

    ros::ServiceServer reset;

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
        odometry_publish = n.advertise<nav_msgs::Odometry>("base_odom",1000);

        // from broadcaster
        get_odom = n.subscribe("/robot/pose", 1000, &IntegrationNode::setOdomCallback, this);

        node_integrator.setMethod(EULER);
        
        //NON SO A COSA SERVA _1
        dynServer.setCallback(boost::bind(&dynamic_Reconfigure_CallBack, &integrator_enum, _1, _2));

        reset = n.advertiseService<integrator::Odom_Reset::Request,
            integrator::Odom_Reset::Response>("reset", boost::bind(&IntegrationNode::odomResetCallback, _1, _2));
    }

    static void setMethod(Method method) {
        ROS_INFO("Selected integration method: %s", method==EULER ? "euler" : "Runge-Kutta");
        node_integrator.setMethod(method);
    }

    void main_loop() {
        ros::spin();
    }

    void integrationCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        // load velocity data from cmd_vel topic message into velocity Matrix
        loadVelocity(msg);

        // set next timeStamp for integrator (to determine integration period) and integrate velocity
        node_integrator.setTimeStamp(msg->header.stamp.toSec()) << velocity;
        position = node_integrator.getPosition();

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

        // from odom broadcaster
        odometry_tf.header.stamp    = odometry_msg.header.stamp     = msg->header.stamp;
        odometry_tf.header.seq      = odometry_msg.header.seq       = msg->header.seq;

        // publish both on the odom topic and send transform
        odometry_publish.publish(odometry_msg);
        tf_broadcaster.sendTransform(odometry_tf);
    }

    void setOdomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
        double roll, pitch, yaw;
        cx += msg->pose.position.x/treshold; cy += msg->pose.position.y/treshold;

        // filter keeping yaw only
        tf2::fromMsg(msg->pose.orientation, q);
        q.normalize();
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        ct += yaw/treshold;
        tTresh--;
        if(tTresh == 0) {
            // generate odometry and transform message header
            odometry_tf.header.frame_id = odometry_msg.header.frame_id  = "world";
            odometry_tf.child_frame_id  = odometry_msg.child_frame_id   = "odom";

            setXYT(cx, cy, ct);
            
            // reset values
            tTresh = treshold; cx = cy = ct = 0;

            // save resources
            get_odom.shutdown();
        }
    }
    
    static void dynamic_Reconfigure_CallBack(int* method,integrator::dinIntegratorConfig &config, int level){
        setMethod((Method)config.integrator_enum);
    }

    static void setXYT(double x, double y, double theta) {

        ROS_INFO("Resetting odometry to: %lf, %lf, %lf", x, y, theta);

        // set position
        odometry_tf.transform.translation.x = odometry_msg.pose.pose.position.x = x;
        odometry_tf.transform.translation.y = odometry_msg.pose.pose.position.y = y;
        odometry_tf.transform.translation.z = odometry_msg.pose.pose.position.z = 0;

        // only keep quaternion's yaw
        q.setRPY(0, 0, theta);
        odometry_tf.transform.rotation = odometry_msg.pose.pose.orientation = tf2::toMsg(q);
        
    }

    // position reset
    static bool odomResetCallback(integrator::Odom_Reset::Request &req,
            integrator::Odom_Reset::Response &res) {
        node_integrator.resetPosition();
        setXYT(req.new_x, req.new_y, req.new_theta);
        return true;
    }

};

int main(int argc, char* argv[]) {
    if(argc < 2) {
        for(int i=0; i<= argc; i++){
            printf("argv[%i] is: %s", i, argv[i]);
        }
        return 1;
    }
    treshold = std::atol(argv[1]);
    tTresh = treshold;
    cx = cy = ct = 0;
    ros::init(argc, argv, "integration_node");
    IntegrationNode integrationNode;
    node_integrator.setTimeStamp(ros::Time::now().toSec());
    integrationNode.main_loop();
    return 0;
}