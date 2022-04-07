#ifndef INTEGRATOR_HPP_
#define INTEGRATOR_HPP_

#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"

#include <math.h>

enum Method{EULER,RUNGE_KUTTA};


extern double zeroPose[3];
extern double set_base_int[9];

class Integrator {
    unsigned int counter;
    Matrix* state;
    Matrix* dir_k;
    Matrix* base_int;
    Method method;
    inline void setX(double x) { return state->set(0, 0,x); };
    inline void setY(double y) { return state->set(1, 0,y); };
    inline void setT(double z) { return state->set(2, 0,z); };
    double offset(Matrix* ticks);
    void setAngle(Matrix* ticks);
public:
    inline double getX() { return state->get(0, 0); };
    inline double getY() { return state->get(1, 0); };
    inline double getT() { return state->get(2, 0); };
    // specify which direct kinematic rule to use
    Integrator(Method method);
    ~Integrator();
    void dirKin(unsigned int h, unsigned int w, double mat[]);
    void inline setPos(double t[]) { this->state->fill(t); }
    void inline resetPos() { setPos(zeroPose); }
    Matrix* integrate(Matrix* ticks);
};

#endif//INTEGRATOR_HPP_