#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

class OdomReset {

    ros::NodeHandle n;
    tf2::Quaternion q;

    ros::Subscriber base_odom;
    double base_odom_val[3];
    bool base_odom_recieved = false;

    ros::Subscriber position_offset;
    double position_offset_msg[3];
    bool position_offset_recieved = false;

    double trash;

public:

    OdomReset() {
        base_odom = n.subscribe("base_odom", 1000, &OdomReset::baseOdomCallback, this);
        position_offset = n.subscribe("odom", 1000, &OdomReset::positionOffsetCallback, this);
    }

    void msgToMem(double* mem, const nav_msgs::Odometry::ConstPtr &msg) {

        // set position
        mem[0] = msg->pose.pose.position.x;
        mem[1] = msg->pose.pose.position.y;

        // set yaw
        tf2::fromMsg(msg->pose.pose.orientation, q);
        q.normalize();
        tf2::Matrix3x3 m(q);
        m.getRPY(trash, trash, mem[2]);
    }

    void baseOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

        msgToMem(base_odom_val, msg);

        if(position_offset_recieved)
            advertisePosition();
        base_odom_recieved = true;
    }

    void positionOffsetCallback(const nav_msgs::Odometry::ConstPtr &msg) {

        msgToMem(base_odom_val, msg);

        if(base_odom_recieved)
            advertisePosition();
        position_offset_recieved = true;
    }

    void advertisePosition() {
        // TODO: write actual transform
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "odom_reset_node");
}