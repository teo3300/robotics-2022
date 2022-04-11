
#include "../odometry/odometry.hpp"

Odometry::Odometry(SpeedCalculator& speedCalculator, Integrator& integrator)
    : speedCalculator(&speedCalculator), integrator(&integrator){}

Odometry::~Odometry(){}

Matrix Odometry::operator<< (Matrix inputs){

    Matrix ret(inputs >> *speedCalculator >> *integrator);

    return ret;
}