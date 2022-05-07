#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include <sensor_msgs/JointState.h>

#include "geometry/odometry.hpp"
#include "proj_const/constants.hpp"
#include "inverse_rpm/Wheels_Rpm.h"
#include "ros/subscriber.h"
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double clean[4] = {0,0,0,0};

Matrix inverse_matrix(4,3);
Matrix result(4,1,clean);

ros::Subscriber sub;
ros::Publisher pub;
ros::Subscriber bag;
ros::Publisher bagpub;

class AvgQueue {
    double avgVal[4];
    double** queue = nullptr;
    int windowSize = 0;
    int curr = 0;

    void make(int windowSize) {
        this->windowSize = windowSize;
        queue = new double*[windowSize];
        for(int i=0; i<windowSize; i++) {
            queue[i] = new double[4];
        }
    }

    void del() {
        for(int i=0; i<windowSize; i++) {
            delete[] &queue[i];
        }
        delete[] queue;
    }
public:

    AvgQueue(int windowSize) {
        make(windowSize);
    }

    ~AvgQueue() {
        del();
    }

    void setSize(int windowSize) {
        del();
        make(windowSize);
    }

    void push(double val[4]) {
        for(int i=0; i<4; i++) {
            queue[curr][i] = val[i];
        }
        curr = (curr+1)%windowSize;
    }

    void doAvg() {
        for(int j=0; j<4; j++) {
            double tmp = 0;
            for(int i=0; i<windowSize; i++) {
                tmp += queue[i][j];
            }
            avgVal[j] = tmp/windowSize;
        }
    }

    inline double getAvg(unsigned i) { return avgVal[i]; }
} avgQueue(0);

void inverseKinCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg){
    //getting values from topic cmd_vel
    double values[3];
    values[0] = msg->twist.linear.x;
    values[1] = msg->twist.linear.y;
    values[2] = msg->twist.angular.z;
    Matrix pose(3,1,values);
    //performing inverse computation
    result = inverse_matrix * pose;

    // sliding window avg denoise
    double tmp[4];
    result.dump(tmp);
    avgQueue.push(tmp);
    avgQueue.doAvg();

    inverse_rpm::Wheels_Rpm wheel_msg;
    // generating mesage header
    wheel_msg.header.stamp = msg->header.stamp;
    wheel_msg.header.frame_id = msg->header.frame_id;
    wheel_msg.header.seq = msg->header.seq;

    //genererating and sending msg
    wheel_msg.rpm_fl=avgQueue.getAvg(0);
    wheel_msg.rpm_fr=avgQueue.getAvg(1);
    wheel_msg.rpm_rl=avgQueue.getAvg(2);
    wheel_msg.rpm_rr=avgQueue.getAvg(3);
    pub.publish(wheel_msg);
    result.fill(clean);
}

int main(int argc, char *argv[]){

    if(argc < 2) {
        avgQueue.setSize(1);
    } else {
        avgQueue.setSize(atoi(argv[1]));
    }

    ros::init(argc, argv,"inverse_kinematic_calculator");

    if(!load_parameters(argc,argv)) return 1;

    ros::NodeHandle node_Handle;

    double inverse_matrix_values [WHEELS*VARS];
    
    generateInverseKinematic(inverse_matrix_values,INVERSE_SCA);
   
    inverse_matrix.fill(inverse_matrix_values);
   
    sub = node_Handle.subscribe("cmd_vel",1000,inverseKinCallBack);
    pub = node_Handle.advertise<inverse_rpm::Wheels_Rpm>("wheels_rpm",1000);
    
    ros::spin();

    return 0;
}
