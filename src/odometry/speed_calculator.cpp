#include "speed_calculator.hpp"

SpeedCalculator::SpeedCalculator(Matrix kinematic) {
    speed = new Matrix(3,1);
    this->kinematic = new Matrix(kinematic);
}

SpeedCalculator::~SpeedCalculator() {
    delete speed;
    delete kinematic;
}

Matrix SpeedCalculator::operator<< (Matrix input) {

    *speed = *kinematic * input;

    Matrix ret(*speed);

    return ret;

}