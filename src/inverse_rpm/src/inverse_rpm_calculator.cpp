#include "ros/node_handle.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#include "../../geometry/include/geometry/odometry.hpp"
#include "../../proj_const/include/proj_const/constants.hpp"
#include "inverse_rpm/Wheels_Rpm.h"

Matrix inverse_matrix(4,3);
Matrix result(4,1);

void inverseKinCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    double* values = new double(3);
    values[0] = msg->pose.pose.position.x;
    values[1] = msg->pose.pose.position.y;
    values[2] = msg->pose.pose.position.z;
    Matrix pose(3,1,values);
    result = inverse_matrix * pose;
    inverse_rpm::Wheels_Rpm wheel_msg;
    //creazione del messaggio da finire
}

int main(int argc, char *argv[]){
    ros::init(argc, argv,"inverse_kinematic_calculator");

    ros::NodeHandle node_Handle;

    double inverse_matrix_values [WHEELS*VARS];
    generateInverseKinematic(inverse_matrix_values,INVERSE_SCA);
    inverse_matrix.fill(inverse_matrix_values);


    ros::Subscriber sub = node_Handle.subscribe("odom",1000,inverseKinCallBack);
    ros::Publisher pub = node_Handle.advertise<inverse_rpm::Wheels_Rpm>("wheels_rpm",1000);

    ros::spin();

    return 0;
}
