#include "odometry.hpp"
#include <cstdio>
#include <cstdlib>
#include <iostream>

double clean[WHEELS*VARS] = {0,0,0,0,0,0,0,0,0,0,0,0};

Odometry::Odometry(Source source){
    this->source = source;
    this->state = new Matrix(4,1,clean);
    this->result = new Matrix(3,1,clean);
}

Odometry::~Odometry(){
    delete state;
    delete result;
}

void Odometry::compute(){
    double* defined_matrix;

    if(this->source==ANG_VEL){
        //matrice di trasformazione con velocità angolare
        defined_matrix = dir_kin_mat;
    } else {
        //matrice di trasformazione con ticks
        defined_matrix = dis_dir_kin_mat;
    }
    //DEBUG
    Matrix* matrix = new Matrix(VARS,WHEELS,defined_matrix);
    /*
    DEBUG
     std::cout << "Matrice matrix : "<<"\n"<<(*matrix)<<"\n";
     std::cout << "Matrice state : "<<"\n"<<(*this->state)<<"\n";
     */
    (*this->result) = (*matrix) * (*this->state);
    delete matrix;
}