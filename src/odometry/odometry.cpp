#include "odometry.hpp"
#include "geometry/matrix.hpp"


Odometry::Odometry(Matrix kinematic) {
    speed = new Matrix(3,1);
    this->kinematic = new Matrix(kinematic);
}

Odometry::~Odometry() {
    delete speed;
    delete kinematic;
}

void Odometry::resetKinematic(Matrix newKinematic){
    delete kinematic; kinematic = new Matrix(newKinematic);
}

Matrix Odometry::getSpeed() {
    return Matrix(*speed);
}

Matrix Odometry::operator<< (Matrix input) {

    *speed = *kinematic * input;

    Matrix ret(*speed);

    return ret;

}