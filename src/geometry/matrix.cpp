#include "matrix.hpp"

inline unsigned int Matrix::cmpt(unsigned int i, unsigned int j){
    return j+i*width;
}

inline void Matrix::set(unsigned int i, unsigned int j, double value){
    dat[cmpt(i,j)] = value;
}

Matrix::Matrix(unsigned int height, unsigned int width){
    this->width = width;
    this->height = height;
    this->dat = new double[height*width];
}

Matrix::~Matrix(){
    delete[] dat;
}

inline unsigned int Matrix::getWidth(){
    return this->width;
}

inline unsigned int Matrix::getHeitht(){
    return this->height;
}

inline double Matrix::get(unsigned int i, unsigned int j){
    return dat[cmpt(i, j)];
}

void Matrix::fill(double* t){
    for(int i=0; i<width*height; i++){
        dat[i] = t[i];
    }
}

static Matrix* mul(Matrix* result, Matrix* a, Matrix* b) {
    if(a->getWidth() != b->getHeitht()){
        return __null;
    }   // valid return only if operation is valid

    double buffer;

    for(int i=0; i<a->getHeitht(); i++){
        for(int j=0; j<b->getWidth(); j++){
            buffer = 0;
            for(int t=0; t<a->getWidth(); t++){
                buffer += a->get(i,t)*b->get(t,j); 
            }   // compute element i,j
            result->set(i, j, buffer);
        }
    }
    return result;
}

std::ostream& operator<<(std::ostream &strm, Matrix &mat) {
    std::string buffer = "";
    for(int i=0; i<mat.getHeitht(); i++){
        for(int j=0; j<mat.getWidth(); j++){
            buffer += std::to_string(mat.get(i, j)) + " ";
        }
        buffer += i==mat.getHeitht()-1 ? "" : "\n";
    }
    return strm << buffer;
}