#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define WHEELS  4
#define VARS    3

#define RADIUS  0.07
#define LENGTH  0.2
#define WIDTH   0.169
#define RATIO   5.0

// TODO: get period definition
#define PERIOD  1
// TODO: get encoder ticks
#define E_BITS  12

// imported from math.c
#define PI      3.14159265358979323846
#define PI_2 	(PI / 2)
#define PI_4 	(PI / 4)
#define _1_PI 	0.318309886183790671538
#define _2_PI 	(_1_PI * 2)

#define T_ROUND (1<<E_BITS)
#define A_TICK  (1*PI/T_ROUND)

extern double dir_kin_mat[
    WHEELS * VARS];

extern double discrete_dir_kin_mat[
    WHEELS * VARS];

extern double inv_kin_mat[
    VARS * WHEELS];

#endif//CONSTANTS_H_