#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <ostream>
#include <stdexcept>
#include <string>

typedef double culo;

class Matrix {
    unsigned int height;
    unsigned int width;
    double* dat;

    inline unsigned int cmpt(unsigned int i, unsigned int j) const { return j+i*width; }

public:
    Matrix(unsigned int height, unsigned int width);
    Matrix(unsigned int height, unsigned int width, double val[]);
    Matrix(Matrix &src);
    ~Matrix();

    inline unsigned int getWidth() const { return this->width; }
    inline unsigned int getHeight() const { return this->height; }
    inline double get(unsigned int i, unsigned int j)const{ return dat[cmpt(i, j)]; }
    inline void set(unsigned int i, unsigned int j, double value){ dat[cmpt(i,j)] = value; }
    inline double sca(Matrix mat) { return get(0,0); }
    void fill(double t[]);
    Matrix* sum(Matrix* a, Matrix* b);
    Matrix* mul(Matrix* a, Matrix* b);
    Matrix* mul(double sca, Matrix mat);
    inline void mult(double sca) { mul(sca, *this); }
    Matrix operator = (const Matrix& other);
    const Matrix operator + (const Matrix& other) const;
    const Matrix operator * (const Matrix& other) const;
    const Matrix operator * (double sca) const;
};

inline Matrix operator*(double sca, const Matrix& mat){
    Matrix tmp(mat.getHeight(), mat.getWidth());
    tmp = mat * sca;
    return tmp;
};
std::ostream& operator<<(std::ostream &strm, Matrix &mat);

#endif//MATRIX_HPP_