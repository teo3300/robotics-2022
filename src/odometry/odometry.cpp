#include "odometry.hpp"


Odometry::Odometry(Matrix kinematic) {
    this->kinematic = new Matrix(kinematic);
}

Odometry::~Odometry() {
    delete kinematic;
}

Matrix Odometry::operator<< (Matrix input) {

    Matrix speed(*kinematic * input);

    return speed;

}