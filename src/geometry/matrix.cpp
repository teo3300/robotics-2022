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

/* Functions omitted
 *
 *
Matrix* Matrix::sum(Matrix* a, Matrix* b){
    if(this->differ(*a) | this->differ(*b)){
        throw std::invalid_argument("Error in vector sum: different size");
    }

    for(int i=0; i<this->height*width; i++){
        this->dat[i] = a->dat[i]+b->dat[i];
    }
    return this;
}

Matrix* Matrix::mul(Matrix* a, Matrix* b) {
    if(a->notMult(*b)){
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
*/

Matrix Matrix::operator = (const Matrix& other){
    if(this->differ(other))
        throw std::invalid_argument("Assignment between different size matrix");
    this->width = other.width;
    this->height = other.height;
    this->fill(other.dat);
    return *this;
}

const Matrix Matrix::operator + (const Matrix& other) const {
    if(this->differ(other)){
        throw std::invalid_argument("Error in matrix sum: different size");
    }

    Matrix tmp(this->height,this->width);
    for(int i=0; i<this->width*height; i++){
        tmp.dat[i] = this->dat[i]+other.dat[i];
    }

    return tmp;
}

const Matrix Matrix::operator * (const Matrix& other) const {
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

const Matrix Matrix::operator * (double sca) const {
    Matrix tmp(this->height, this->width);
    for(int i=0; i<width*height; i++){
        tmp.dat[i] = dat[i]*sca;
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