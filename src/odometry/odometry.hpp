#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"

#include <math.h>

enum Source{ANG_VEL, TICKS};

class Odometry {
    Matrix* state;
    Matrix* kinematic;

public:
    Odometry(unsigned int h, unsigned int w, double kinematic[]);
    ~Odometry();
    Matrix operator<< (Matrix src);
};

#endif//ODOMETRY_HPP_