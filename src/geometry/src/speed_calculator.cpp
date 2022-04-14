#include "geometry/speed_calculator.hpp"
#include <iostream>

SpeedCalculator::SpeedCalculator(Matrix kinematic, ComputeType computeType)
        : computeType(computeType) {
    currStamp = 0;
    prevStamp = 0;
    speed = new Matrix(3,1);
    this->kinematic = new Matrix(kinematic);
}

SpeedCalculator::~SpeedCalculator() {
    delete speed;
    delete kinematic;
}

Matrix SpeedCalculator::operator<< (Matrix input) {

    *speed = *kinematic / (computeType == DISCRETE ? getDt() : 1) * input;

    Matrix ret(*speed);

    return ret;

}