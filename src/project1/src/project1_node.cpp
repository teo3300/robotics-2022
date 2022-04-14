#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "geometry/odometry.hpp"
#include "proj_const/constants.hpp"
#include "geometry_msgs/PoseStamped.h"

double reset[] = {0,0,0,0};

// input matrices
Matrix a_input(4,1);
Matrix t_input(4,1);

// previous tick matrix (to differentiate)
Matrix t_prev_(4,1);

// speedCompute
SpeedCalculator a_speedCompute(Matrix(3, 4, rpm_dir_kin_mat), CONTINUE);
SpeedCalculator t_speedCompute(Matrix(3, 4, dis_dir_kin_mat), DISCRETE);

// calculate speed with both angular velocity and ticks
Matrix a_speed(3,1);
Matrix t_speed(3,1);

// integrate
Integrator a_integrator(RUNGE_KUTTA);
Integrator t_integrator(RUNGE_KUTTA);

// matrices to store robot position
Matrix a_position(3,1);
Matrix t_position(3,1);

void movCallBack(const sensor_msgs::JointState::ConstPtr& msg) {

    // determine integration period
    double timeStamp = msg->header.stamp.toSec();

    // get the timestamp of each message for debug
    ROS_INFO("/wheel/states: msg at %lf", timeStamp);

    // fills tikcs and angular veolcity matrix with respective data
    a_input.fill(msg->velocity.data());
    t_input.fill(msg->position.data());

    // print matrices content
    std::cout << "wheels angular speed is:\n" << a_input << "\n";
    std::cout << "wheels position is:\n" << t_input << "\n\n";

    // compute speed using both angular velocity and delta_ticks
    a_speed = a_speedCompute /*timestamp not needed*/ << a_input;
    t_speed = t_speedCompute.setTimeStamp(timeStamp)  << (t_input - t_prev_);

    // print matrices content (also print module of the vector difference)
    std::cout << "speed given angular is:\n" << a_speed << "\n";
    std::cout << "speed given ticks is:\n" << t_speed << "\n";
    std::cout << "|ERROR| is: " << ~(t_speed - a_speed) << "\n\n";

    // integrate velocities
    a_position = a_integrator.setTimeStamp(timeStamp) << a_speed;
    t_position = t_integrator.setTimeStamp(timeStamp) << t_speed;

    // print matrices content (also print module of the vector difference)
    std::cout << "position given angular is:\n" << a_position << "\n";
    std::cout << "position given ticks is:\n" << t_position << "\n";
    std::cout << "|ERROR| is: " << ~(t_position - a_position) << "\n";
    std::cout << "===================================================\n";

    // switch to next step
    t_prev_ = t_input;

}

int main (int argc, char* argv[]) {

    t_input.fill(reset);
    t_prev_.fill(reset);

    ros::init(argc, argv, "project1_node");

    ros::NodeHandle n;

    ros::Subscriber mov = n.subscribe("/wheel_states", 1000, movCallBack);

    ros::spin();

    return 0;
}