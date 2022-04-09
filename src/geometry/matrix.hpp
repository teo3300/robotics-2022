#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <ostream>
#include <stdexcept>
#include <string>

class Matrix {
    unsigned int height;
    unsigned int width;
    double* dat;

    inline unsigned int cmpt(unsigned int i, unsigned int j) const { return j+i*width; }

    inline int notMult(Matrix other) { return other.height - this->width;}
    inline int diffWidth(Matrix other) { return other.width - this->width; }
    inline int diffHeight(Matrix other) { return other.width - this->width; }
    inline int differ(Matrix other) {
        return this->diffHeight(other) | this->diffWidth(other);
    }

public:
    Matrix();
    Matrix(const Matrix &src);
    Matrix(unsigned int height, unsigned int width);
    Matrix(unsigned int height, unsigned int width, double val[]);
    ~Matrix();

    inline unsigned int getWidth() { return this->width; }
    inline unsigned int getHeight() { return this->height; }
    inline double get(unsigned int i, unsigned int j) { return (*this)(i,j); }

    inline void set(unsigned int i, unsigned int j, double value){ (*this)(i,j) = value; }
    inline double sca(Matrix mat) { return (*this)(0,0); }
    void fill(double t[]);

    // I have no fucking Idea what I'm doing
    double& operator()(unsigned y, unsigned x);
    const double &operator()(unsigned y, unsigned x) const;

    Matrix operator = (Matrix other);
    Matrix operator + (Matrix other);
    Matrix operator * (Matrix other);
    Matrix operator * (double sca);
    Matrix operator ! ();
};

inline Matrix operator*(double sca, Matrix mat){
    return mat * sca;
};

std::ostream& operator<<(std::ostream &strm, Matrix &mat);

#endif//MATRIX_HPP_