#include "odometry.hpp"

static double clean[WHEELS*VARS] = {0,0,0,0,0,0,0,0,0,0,0,0};

Odometry::Odometry(unsigned int h, unsigned int w, double kinematic[]){
    this->state = new Matrix(3,1);
    this->kinematic = new Matrix(h,w,kinematic);
}

Odometry::~Odometry(){
    delete state;
    delete kinematic;
}

Matrix Odometry::operator<< (Matrix src) {

    (*state) = (*kinematic) * (src);

    Matrix ret(*state);

    return ret;
}