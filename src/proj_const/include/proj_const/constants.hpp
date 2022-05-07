#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <ros/ros.h>

// imported from math.c
#define ROS_PI      (3.14159265358979323846)
#define ROS_PI_2 	(ROS_PI / 2)
#define ROS_PI_4 	(ROS_PI / 4)
#define ROS__1_PI 	(0.318309886183790671538)
#define ROS__2_PI 	(ROS__1_PI * 2)

#define SETTABLE    5
#define WHEELS      4
#define VARS        3

typedef enum {
    WHEEL_RADIUS = 0,
    ROBOT_LENGTH,
    ROBOT_WIDT,
    ROBOT_CPR,
    GEAR_RATIO,
    param_enum_values
} param_enum;

typedef union {
    double list[param_enum_values];
    struct {
        double
            wheel_radius,
            robot_length,
            robot_width,
            robot_cpr,
            gear_ratio;
    } nominal;
} Parameters;

#define RADIUS   (robot_parameters.nominal.wheel_radius)
#define LENGTH   (robot_parameters.nominal.robot_length)
#define WIDTH    (robot_parameters.nominal.robot_width)
#define CPR      (robot_parameters.nominal.robot_cpr)
#define RATIO    (robot_parameters.nominal.gear_ratio)

#define L      (LENGTH + WIDTH)

#define DIRECT_RADIANT_SEC_SCA  (RADIUS/(4 * RATIO))
#define DIRECT_RADIANT_MIN_SCA  (DIRECT_RADIANT_SEC_SCA / 60.0)
#define DIRECT_ROUND_MIN_SCA    (DIRECT_RADIANT_SEC_SCA * (2 * ROS_PI) / 60.0)
#define DIRECT_DISCRETE_SCA     (DIRECT_RADIANT_SEC_SCA * 2 * ROS_PI / CPR)
#define INVERSE_SCA             (RATIO * 60.0 / RADIUS)

Parameters robot_parameters;

const double dir_kin_template[WHEELS * VARS] = {
     1,   1,   1,   1,
    -1,   1,   1,  -1,
    -1,   1,  -1,   1
};

const double inv_kin_template[VARS * WHEELS] = {
    1,  -1,  -1,
    1,   1,   1,
    1,   1,  -1,
    1,  -1,   1
};

bool load_parameters(int argc, char* argv[]) {

    ros::NodeHandle param_n;
    if(!param_n.getParam("/par_wheel_radius", RADIUS)) return false;
    if(!param_n.getParam("/par_robot_length", LENGTH)) return false;
    if(!param_n.getParam("/par_robot_width", WIDTH)) return false;
    if(!param_n.getParam("/par_cpr", CPR)) return false;
    if(!param_n.getParam("/par_gear_ratio", RATIO)) return false;

    return true;
}

void generateDirectKinematic(double* mem, double scalar) {
    for(int i=0; i<WHEELS*VARS; i++) {
        mem[i] = dir_kin_template[i]*scalar;
        if(i >= WHEELS * (VARS-1)) mem[i] = mem[i] / L;
    }
}
void generateInverseKinematic(double* mem, double scalar) {
    for(int i=0; i<WHEELS*VARS; i++) {
        mem[i] = inv_kin_template[i]*scalar;
        if(!((i+1) % VARS)) mem[i] = mem[i] * L;
    }
}

#endif//CONSTANTS_H_