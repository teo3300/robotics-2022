#include "matrix.hpp"
#include "string.h"
#include <iostream>
#include <stdexcept>

Matrix::Matrix(unsigned int height, unsigned int width)
        : width(width), height(height){
    dat = new double[width*height];
};

Matrix::Matrix(unsigned int height, unsigned int width, double val[])
        : Matrix(height, width){
    fill(val);
}

Matrix::Matrix(const Matrix &src)
        : Matrix(src.height,src.width){
    fill(src.dat);
}

Matrix::~Matrix(){
    delete[] dat;
}

inline void Matrix::fill(double t[]){
    memcpy(this->dat, t, width*height*sizeof(double));
}

Matrix Matrix::operator = (Matrix other) {
    if(this->differ(other))
        throw std::invalid_argument("Assignment between different size matrix");
    this->width = other.width;
    this->height = other.height;
    this->fill(other.dat);
    return *this;
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

    Matrix tmp(this->height,other.width);
    double buffer;
    for(int i=0; i<this->height; i++){
        for(int j=0; j<other.width; j++){
            buffer = 0;
            for(int t=0; t<this->width; t++){
                buffer += this->get(i,t)*other.get(t, j); 
            }
            tmp.set(i, j, buffer);
        }
    }
    return tmp;
}

Matrix Matrix::operator * (double sca) const {
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
            tmp.set(j,i,get(i,j));
        }
    }
    return tmp;
}

std::ostream& operator<<(std::ostream &strm, Matrix &mat) {
    std::string buffer = "";
    for(int i=0; i<mat.getHeight(); i++){
        for(int j=0; j<mat.getWidth(); j++){
            buffer += std::to_string(mat.get(i, j)) + " ";
        }
        buffer += i==mat.getHeight()-1 ? "" : "\n";
    }
    return strm << buffer;
}