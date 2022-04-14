#ifndef INTEGRATOR_HPP_
#define INTEGRATOR_HPP_

#include "../matrix/matrix.hpp"

enum Method{EULER,RUNGE_KUTTA};


extern double zeroPose[3];
extern double set_base_int[9];

class Integrator {
    Method method;
    double period;
    double angularSpeed;
    Matrix* state;
    Matrix* base_int;

    inline double getX() { return (*state)(0, 0); }
    inline double getY() { return (*state)(1, 0); }
    inline double getT() { return (*state)(2, 0); }
    
    inline void setX(double x) { (*state)(0, 0) = x; }
    inline void setY(double y) { (*state)(1, 0) = y; }
    inline void setT(double z) { (*state)(2, 0) = z; }

    inline double getPeriod() { return period; }
    
    inline double offset() { return angularSpeed * period / 2; }
    void setAngle();
public:
    // Runge kutta integration
    Integrator(Method method, double period);
    ~Integrator();

    inline void setMethod(Method method) { this->method = method; }
    inline void resetPosition(Matrix position) { *state = position; }
    inline Matrix getPosition() { return Matrix(*state); }

    Matrix operator<< (Matrix speed);
};

inline Matrix operator>> (Matrix speed, Integrator& integrator) {
    return integrator << speed;
}

#endif//INTEGRATOR_HPP_