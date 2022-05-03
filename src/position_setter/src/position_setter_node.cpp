#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define GT_NC 4
#define HZ 10

unsigned long treshold;
unsigned long tTresh;
double cx, cy, ct;

class PositionSetter {

    const char *name = "position_setter";

    ros::NodeHandle n;

    ros::Subscriber get_odom;
    ros::Subscriber got_integrated;

    ros::Publisher odometry_publish;
    nav_msgs::Odometry odometry_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped odometry_tf;

public:

    PositionSetter() {

        // TODO: only use this for first position setting, use current robot position for further odometry reset
        get_odom = n.subscribe("/robot/pose", 1000, &PositionSetter::setOdomCallback, this);
        got_integrated = n.subscribe("odom", 1000, &PositionSetter::odomBroadcastCallback, this);

        odometry_publish = n.advertise<nav_msgs::Odometry>("base_odom",1000);
    }

    void main_loop() {
        ros::spin();
    }

    void odomBroadcastCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odometry_tf.header.stamp    = odometry_msg.header.stamp     = ros::Time::now();
        odometry_tf.header.seq      = odometry_msg.header.seq       = msg->header.seq;

        // publish both on the odom topic and send transform
        odometry_publish.publish(odometry_msg);
        tf_broadcaster.sendTransform(odometry_tf);
    }

    void setOdomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
        tf2::Quaternion q;
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

            // set initial position
            odometry_tf.transform.translation.x = odometry_msg.pose.pose.position.x = cx;
            odometry_tf.transform.translation.y = odometry_msg.pose.pose.position.y = cy;
            odometry_tf.transform.translation.z = odometry_msg.pose.pose.position.z = 0;

            // only keep quaternion's yaw
            q.setRPY(0, 0, ct);

            ROS_INFO("Node %s: Requested new odometry computation: setting at %lf, %lf, %lf", name, cx, cy, ct);

            // set initial positon
            odometry_tf.transform.rotation = odometry_msg.pose.pose.orientation = tf2::toMsg(q);
            
            // reset values
            tTresh = treshold; cx = cy = ct = 0;

            // save resources
            get_odom.shutdown();
        }
    }

};

int main(int argc, char* argv[]){
    if(argc < 2) {
        for(int i=0; i<= argc; i++){
            ROS_INFO("argv[%i] is: %s", i, argv[i]);
        }
        return 1;
    }
    treshold = std::stoul(argv[1]);
    tTresh = treshold;
    cx = cy = ct = 0;
    ros::init(argc, argv, "position_setter_node");
    PositionSetter positionSetter;
    positionSetter.main_loop();
    return 0;
}
