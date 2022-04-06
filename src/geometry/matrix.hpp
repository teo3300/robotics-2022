#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <ostream>
#include <string>

class Matrix {
    unsigned int height;
    unsigned int width;
    double* dat;

    inline unsigned int cmpt(unsigned int i, unsigned int j);

public:
    Matrix(unsigned int height, unsigned int width);
    ~Matrix();

    inline unsigned int getWidth();
    inline unsigned int getHeitht();
    inline double get(unsigned int i, unsigned int j);
    inline void set(unsigned int i, unsigned int j, double value);
    void fill(double* t);
    static Matrix* mul(Matrix* result, Matrix* a, Matrix* b);
};

std::ostream& operator<<(std::ostream &strm, Matrix &mat);

#endif//MATRIX_HPP_