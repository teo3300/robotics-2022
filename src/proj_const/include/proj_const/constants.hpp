#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// imported from math.c
#define PI      3.14159265358979323846
#define PI_2 	(PI / 2)
#define PI_4 	(PI / 4)
#define _1_PI 	0.318309886183790671538
#define _2_PI 	(_1_PI * 2)

#define SETTABLE    5
#define WHEELS      4
#define VARS        3

typedef union {
    double list[SETTABLE];
    struct {
        double
            wheel_radius,
            robot_length,
            robot_width,
            robot_cpr,
            gear_ratio;
    } nominal;
} Parameters;

extern Parameters robot_parameters;

#define RADIUS (robot_parameters.nominal.wheel_radius)
#define LENGTH (robot_parameters.nominal.robot_length)
#define WIDTH  (robot_parameters.nominal.robot_width)
#define CPR    (robot_parameters.nominal.robot_cpr)
#define RATIO  (robot_parameters.nominal.gear_ratio)

#define L      (LENGTH + WIDTH)

#define DIRECT_RADIANT_SEC_SCA  (RATIO*RADIUS/4)
#define DIRECT_RADIANT_MIN_SCA  ((DIRECT_RADIANT_SEC_SCA / 60))
#define DIRECT_ROUND_MIN_SCA    ((DIRECT_RADIANT_SEC_SCA * (2 * PI) / 60))
#define DIRECT_DISCRETE_SCA     ((DIRECT_RADIANT_SEC_SCA * 2 * PI / CPR))
#define INVERSE_SCA             ((1/RADIUS))

bool load_parameters(int argc, char* argv[]);

void generateDirectKinematic(double* mem, double scalar);
void generateInverseKinematic(double* mem, double scalar);

#endif//CONSTANTS_H_