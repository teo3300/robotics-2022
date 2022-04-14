#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define WHEELS  4
#define VARS    3

#define RADIUS  0.07
#define LENGTH  0.2
#define WIDTH   0.169
#define RATIO   5.0

// TODO: get encoder ticks
#define T_ROUND 64

// imported from math.c
#define PI      3.14159265358979323846
#define PI_2 	(PI / 2)
#define PI_4 	(PI / 4)
#define _1_PI 	0.318309886183790671538
#define _2_PI 	(_1_PI * 2)

#ifdef E_BITS
    #define T_ROUND (1<<E_BITS)
#endif

extern double dir_kin_mat[
    VARS * WHEELS];

extern double rpm_dir_kin_mat[
    VARS * WHEELS];

extern double dis_dir_kin_mat[
    VARS * WHEELS];

extern double inv_kin_mat[
    WHEELS * VARS];

#define RUNGE_KUTTA_OFFSET (RADIUS*PI_2/(T_ROUND*(LENGTH+WIDTH)))

#endif//CONSTANTS_H_