#include "odometry.hpp"
#include <cstdio>
#include <cstdlib>

Odometry::Odometry(Matrix* model){
    this->model = model;
    this->state = new Matrix(this->model->getWidth(),1);
    this->result = new Matrix(this->model->getHeight(),1);
}

Odometry::~Odometry(){
    delete state;
    delete kinematic;
}

void Odometry::compute(){
    (*this->result) = (*this->model) * (*this->state);
}