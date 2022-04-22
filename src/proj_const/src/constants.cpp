#include "proj_const/constants.hpp"

#define CONST_MAT true

#ifndef CONST_MAT
#define CONST_MAT trure
#endif

#if CONST_MAT
    // direct kinematic using radiant/sec as input
    #define DI (RATIO*(RADIUS/4))
    
    // direct kinematic using rpm as input
    #define DR (RATIO*(DI * (2 * PI) / 60))

    // direct kinematic using radiant/min
    #define DM (RATIO*(DI / 60))

    // discrete direct matrix requires knowing time period to determine correct scalar
    #define DD (RATIO*(DI * 2 * PI / T_ROUND))

    // inverse kinematic using wheel angular velocity is defined as
    #define IN (RATIO*(1/RADIUS))

#else
    #define DI  1
    #define DR  1
    #define IN  1
    #define DD  1
#endif

#define L   (LENGTH+WIDTH)
#define LW  (LENGTH+WIDTH)

double dir_kin[WHEELS * VARS] = {
     DI,    DI,    DI,    DI,
    -DI,    DI,    DI,   -DI,
    -DI/L,  DI/L, -DI/L,  DI/L
};

double round_min_dir_kin[WHEELS * VARS] = {
     DR,    DR,    DR,    DR,
    -DR,    DR,    DR,   -DR,
    -DR/L,  DR/L, -DR/L,  DR/L
};

double rad_min_dir_kin[WHEELS * VARS] = {
     DM,    DM,    DM,    DM,
    -DM,    DM,    DM,   -DM,
    -DM/L,  DM/L, -DM/L,  DM/L
};

double dis_dir_kin[WHEELS * VARS] = {
     DD,    DD,    DD,    DD,
    -DD,    DD,    DD,   -DD,
    -DD/L,  DD/L, -DD/L,  DD/L
};

double inv_kin[VARS * WHEELS] = {
     IN, -IN, -IN*LW,
     IN,  IN,  IN*LW,
     IN,  IN, -IN*LW,
     IN, -IN,  IN*LW
};

#undef DI
#undef IN
#undef DD