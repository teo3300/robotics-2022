#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

class TFBroadcaster {
    // communication handling
    ros::NodeHandle n;
    ros::Subscriber sub;

    // tf2 broadcasting
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

public:
    TFBroadcaster() {
        sub = n.subscribe("odom", 1000, &TFBroadcaster::broadcastCallback, this);
    }

    void broadcastCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // replicate message stamp
        transformStamped.header.stamp = msg->header.stamp;

        // set frames
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = msg->header.frame_id;

        // performs translation
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.x = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = 0.0;

        // performs rotation
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        transformStamped.transform.rotation.x= q.x();
        transformStamped.transform.rotation.y= q.y();
        transformStamped.transform.rotation.z= q.z();
        transformStamped.transform.rotation.w= q.w();
        br.sendTransform(transformStamped);
    }

};

int main(int argc, char* argv []) {
    ros::init(argc, argv, "tf_broadcaster");
    TFBroadcaster TFBroadcaster;
    ros::spin();
    return 0;
}