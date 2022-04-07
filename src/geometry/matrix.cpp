#include "matrix.hpp"
#include "string.h"
#include <iostream>
#include <stdexcept>

Matrix::Matrix(unsigned int width, unsigned int height){
    this->width = width;
    this->height = height;
    this->dat = new double[height*width];
}

Matrix::Matrix(unsigned int width, unsigned int height, double val[]){
    this->width = width;
    this->height = height;
    this->dat = new double[height*width];
    fill(val);
}

Matrix::Matrix(Matrix& src){
    this->width = src.getWidth();
    this->height = src.getHeight();
    this->dat = new double[height*width];
    fill(src.dat);
}

Matrix::~Matrix(){
    delete[] dat;
}

inline void Matrix::fill(double t[]){
    memcpy(this->dat, t, width*height*sizeof(double));
}

Matrix* Matrix::sum(Matrix* a, Matrix* b){
    if(this->width != 1 || a->width != 1 || b->width != 1){
        throw std::invalid_argument("Error: can only add vectors");
    }
    if(!(this->height == a->height && this->height == b->height)){
        throw std::invalid_argument("Error in vector sum: different size");
    }

    for(int i=0; i<this->height; i++){
        this->dat[i] = a->dat[i]+b->dat[i];
    }
    return this;
}

Matrix* Matrix::mul(Matrix* a, Matrix* b) {
    if(a->getWidth() != b->getHeight()){
        throw std::invalid_argument("Error in matrix moltiplication: Wrong matrix size");
    }
    double buffer;

    for(int i=0; i<a->getHeight(); i++){
        for(int j=0; j<b->getWidth(); j++){
            buffer = 0;
            for(int t=0; t<a->getWidth(); t++){
                buffer += a->get(i,t)*b->get(t,j); 
            }
            this->set(i, j, buffer);
        }
    }
    return this;
}

Matrix* Matrix::mul(double sca, Matrix mat){
    for(int i=0; i<mat.getWidth()*mat.getHeight(); i++){
        this->dat[i] = sca*mat.dat[i];
    }
    return this;
}
/*
Matrix Matrix::operator = (const Matrix& other){
    if(this->width != other.width || this->height != other.height)
        throw std::invalid_argument("Assignment between different size matrix");
    this->width = other.width;
    this->height = other.height;
    this->fill(other.dat);
    return *this;
}

const Matrix Matrix::operator * (Matrix& other) {
    if(this->width != other.height){
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

const Matrix Matrix::operator * (double other) {
    Matrix tmp(height, width);
    for(int i=0; i<width*height; i++){
        tmp.dat[i] = dat[i]*other;
    }
    return tmp;
}

const Matrix Matrix::operator + (Matrix& other){
    if(this->width != 1 || other.width != 1){
        throw std::invalid_argument("Error: can only add vectors");
    }
    if(this->height != other.height){
        throw std::invalid_argument("Error in vector sum: different size");
    }

    Matrix tmp(this->height,1);
    for(int i=0; i<this->height; i++){
        tmp.dat[i] = this->dat[i]+other.dat[i];
    }

    return tmp;
}
*/

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