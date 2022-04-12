#include "matrix.hpp"
#include "string.h"
#include <iostream>
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

Matrix::~Matrix(){
    delete [] dat;
}

inline void Matrix::fill(double t[]){
    memcpy(dat, t, width*height*sizeof(double));
}

double &Matrix::operator()(unsigned y, unsigned x) { return dat[cmpt(y, x)]; }

const double &Matrix::operator()(unsigned i, unsigned j) const {
  return dat[cmpt(i, j)];
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

    Matrix tmp(this->height,this->width);
    for(int i=0; i<this->width*height; i++){
        tmp.dat[i] = this->dat[i]+other.dat[i];
    }

    return tmp;
}

Matrix Matrix::operator * (Matrix other) {
    if(this->notMult(other)){
        throw std::invalid_argument("Error in matrix moltiplication: Wrong matrix size");
    }

    Matrix tmp(height,other.width);
    double buffer;
    for(int i=0; i<height; i++){
        for(int j=0; j<other.width; j++){
            buffer = 0;
            for(int t=0; t<width; t++){
                buffer += get(i,t)*other(t, j); 
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


Matrix Matrix::operator ! () {
    Matrix tmp(this->width, this->height);
    for(int i=0; i<height; i++){
        for(int j=0; j<width; j++){
            tmp(j,i) = get(i,j);
        }
    }
    return tmp;
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