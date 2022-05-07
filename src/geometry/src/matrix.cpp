#include "geometry/matrix.hpp"
#include "string.h"
#include <iostream>
#include <math.h>
#include <stdexcept>

Matrix::Matrix(unsigned int height, unsigned int width)
        : width(width), height(height) {
    dat = new double[width*height];
}

Matrix::Matrix(unsigned int height, unsigned int width, double val[])
        : height(height), width(width) {
    dat = new double[width*height];
    fill(val);
}

Matrix::Matrix(const Matrix &src)
        : height(src.height), width(src.width){
    dat = new double[height*width];
    fill(src.dat);
}

Matrix::Matrix() : Matrix(1,1) {};

Matrix::~Matrix(){
    delete [] dat;
}

inline void Matrix::fill(const double t[]){
    memcpy(dat, t, width*height*sizeof(double));
}

void Matrix::dump(double* t){
    memcpy(t, dat, width*height*sizeof(double));
}

double &Matrix::operator()(unsigned i) {
    return dat[i];
}

const double &Matrix::operator()(unsigned i) const {
    return dat[i];
}

double &Matrix::operator()(unsigned i, unsigned j) {
    return (*this)(i*width+j);
}

const double &Matrix::operator()(unsigned i, unsigned j) const {
    return (*this)(i*width+j);
}

Matrix Matrix::operator = (Matrix other) { // pass reference to object in a fancy way

    if(this == &other) return *this;

    if(this->differ(other))
        throw std::invalid_argument("Assignment between different size matrix");

    width = other.width;
    height = other.height;
    
    fill(other.dat);

    return *this;   // needed to concatenate assignment
}

Matrix Matrix::operator + (Matrix other) {
    if(this->differ(other)){
        throw std::invalid_argument("Error in matrix sum: different size");
    }

    Matrix tmp(height, width);
    for(int i=0; i<width*height; i++){
        tmp.dat[i] = dat[i]+other.dat[i];
    }

    return tmp;
}


Matrix Matrix::operator - (Matrix other) {
    return (*this) + (-1)*other;
}

Matrix Matrix::operator * (Matrix other) {
    if(this->notMult(other)){
        throw std::invalid_argument("Error in matrix multiplication: Wrong matrix size");
    }

    Matrix tmp(height,other.width);
    double buffer;
    for(int i=0; i<height; i++){
        for(int j=0; j<other.width; j++){
            buffer = 0;
            for(int t=0; t<width; t++){
                buffer += (*this)(i,t)*other(t, j); 
            }
            tmp(i,j) = buffer;
        }
    }
    return tmp;
}

Matrix Matrix::operator * (double sca) {

    Matrix tmp(this->height, this->width);
    for(int i=0; i<width*height; i++){
        tmp.dat[i] = dat[i]*sca;
    }

    return tmp;
}


Matrix Matrix::operator / (double sca) {
    return (*this) * (1/sca);
}


Matrix Matrix::operator ! () {
    Matrix tmp(this->width, this->height);
    for(int i=0; i<height; i++){
        for(int j=0; j<width; j++){
            tmp(j,i) = get(i,j);
        }
    }
    return tmp;
}


double Matrix::operator ~ () {
    if(this->getWidth() != 1 ) {
        throw std::invalid_argument("Error: Norm is only defined on vectors");
    }
    double c=0;
    for(int i=0; i<height; i++) {
        c += dat[i] * dat[i];
    }
    return sqrt(c);
}

std::ostream& operator<<(std::ostream &strm, Matrix &mat) {
    std::string buffer = "";
    for(int i=0; i<mat.getHeight(); i++){
        for(int j=0; j<mat.getWidth(); j++){
            buffer += std::to_string(mat(i, j)) + " ";
        }
        buffer += i==mat.getHeight()-1 ? "" : "\n";
    }
    return strm << buffer;
}