#include "constants.hpp"

#define CONST_MAT true

#ifndef CONST_MAT
#define CONST_MAT false
#endif

#if CONST_MAT
    #define DI (RADIUS/4)
    #define DD (PI_2*RADIUS/T_ROUND)
    #define IN (1/RADIUS)
#else
    #define DI  1
    #define IN  1
    #define DD  1
#endif

#define L   (LENGTH+WIDTH)
#define LW  (LENGTH+WIDTH)

double dir_kin_mat[WHEELS * VARS] = {
     DI,   DI,   DI,    DI,
    -DI,   DI,   DI,   -DI,
    -DI/L, DI/L, -DI/L,  DI/L
};

double dis_dir_kin_mat[WHEELS * VARS] = {
    -DD,   DD,   -DD,    DD,
     DD,   DD,   -DD,   -DD,
     DD/L, DD/L,  DD/L,  DD/L
};

double inv_kin_mat[VARS * WHEELS] = {
    -IN,  IN,  IN*LW,
     IN,  IN,  IN*LW,
    -IN, -IN,  IN*LW,
     IN, -IN,  IN*LW
};

#undef DI
#undef IN
#undef DD