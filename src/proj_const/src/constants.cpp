#include "proj_const/constants.hpp"

#define CONST_MAT true

#ifndef CONST_MAT
#define CONST_MAT trure
#endif

#if CONST_MAT
    // direct kinematic using rad/sec as input
    #define DI (RADIUS/4)
    
    // direct kinematic using rpm as input
    #define DR (DI * (2*PI) / 60)

    // discrete direct matrix requires knowing time period to determine correc scalar
    #define DD (DI * 2 * PI / T_ROUND)

    // inverse kinematic using wheel angular velocity is defined as
    #define IN (1/RADIUS)

#else
    #define DI  1
    #define DR  1
    #define IN  1
    #define DD  1
#endif

#define L   (LENGTH+WIDTH)
#define LW  (LENGTH+WIDTH)

double dir_kin_mat[WHEELS * VARS] = {
     DI,    DI,    DI,    DI,
    -DI,    DI,    DI,   -DI,
    -DI/L, -DI/L,  DI/L,  DI/L
};

double rpm_dir_kin_mat[WHEELS * VARS] = {
     DR,    DR,    DR,    DR,
    -DR,    DR,    DR,   -DR,
    -DR/L, -DR/L,  DR/L,  DR/L
};

double dis_dir_kin_mat[WHEELS * VARS] = {
     DD,    DD,    DD,    DD,
    -DD,    DD,    DD,   -DD,
    -DD/L, -DD/L,  DD/L,  DD/L
};

double inv_kin_mat[VARS * WHEELS] = {
    -IN,  IN, IN*LW,
     IN,  IN, IN*LW,
    -IN, -IN, IN*LW,
     IN, -IN, IN*LW
};

#undef DI
#undef IN
#undef DD