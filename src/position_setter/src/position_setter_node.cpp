#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PositionSetter {

    const char *name = "position_setter";

    ros::NodeHandle n;

    ros::Subscriber robot_position;

    ros::Publisher odometry_publish;
    nav_msgs::Odometry odometry_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped odometry_tf;

    bool must_compute = true;

public:

    PositionSetter() {
        robot_position = n.subscribe("/robot/pose", 1000, &PositionSetter::positionSetterCallBack, this);
        odometry_publish = n.advertise<nav_msgs::Odometry>("base_odom",1000);
    }

    void main_loop() {
        ros::spin();
    }

    void positionSetterCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
        odometry_tf.header.stamp    = odometry_msg.header.stamp     = msg->header.stamp;
        odometry_tf.header.seq      = odometry_msg.header.seq       = msg->header.seq;
        
        if(must_compute) {
            must_compute = false;
            
            // generate odometry and transform message header
            odometry_tf.header.frame_id = odometry_msg.header.frame_id  = "world";
            odometry_tf.child_frame_id  = odometry_msg.child_frame_id   = "odom";

            // set initial position
            odometry_tf.transform.translation.x = odometry_msg.pose.pose.position.x = msg->pose.position.x;
            odometry_tf.transform.translation.y = odometry_msg.pose.pose.position.y = msg->pose.position.y;
            odometry_tf.transform.translation.z = odometry_msg.pose.pose.position.z = 0; // msg->pose.position.z;

            // filter keeping yaw only
            tf2::Quaternion q;
            tf2::fromMsg(msg->pose.orientation, q);
            q.normalize();
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            q.setRPY(0, 0, yaw);

            ROS_INFO("Node %s: Requested new odometry computation: setting at %lf, %lf, %lf", name, msg->pose.position.x, msg->pose.position.y, yaw);

            // set initial positon
            odometry_tf.transform.rotation = odometry_msg.pose.pose.orientation = tf2::toMsg(q);
        }
        // publish both on the odom topic and send transform
        odometry_publish.publish(odometry_msg);
        tf_broadcaster.sendTransform(odometry_tf);
    }

};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "position_setter_node");
    PositionSetter positionSetter;
    positionSetter.main_loop();
    return 0;
}
