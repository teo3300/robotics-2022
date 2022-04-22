#include "geometry/speed_calculator.hpp"
#include <iostream>

SpeedCalculator::SpeedCalculator(ComputeType computeType)
        : computeType(computeType) {
    currStamp = 0;
    prevStamp = 0;
    speed = new Matrix(3,1);
}

SpeedCalculator::~SpeedCalculator() {
    delete speed;
    delete kinematic;
}

void SpeedCalculator::setKinematic (Matrix kinematic) {
    this->kinematic = new Matrix(kinematic);
}

Matrix SpeedCalculator::operator<< (Matrix input) {

    *speed = (*kinematic / (computeType == DISCRETE ? getDt() : 1)) * input;

    Matrix ret(*speed);

    return ret;

}