#include "integrator.hpp"
#include <cmath>

double zeroPose[3] = {0,0,0};
double set_base_int[9] = {0,0,0,0,0,0,0,0,1};

Integrator::Integrator(Method method) : Integrator(method,0) {};

Integrator::Integrator(Method method, double rk_offset)
        : method(method), dir_k(nullptr), counter(0), rk_offset(rk_offset) {
    state = new Matrix(3,1,zeroPose);
    speed = new Matrix(3,1);
    base_int = new Matrix(3,3,set_base_int);
};

Integrator::~Integrator(){
    delete state;
    delete speed;
    delete base_int;
    delete dir_k;
}

void Integrator::dirKin(unsigned int h, unsigned int w, double mat[]) {
    dir_k = new Matrix(h, w, mat);
    signals = w;
}

Matrix Integrator::integrate(Matrix ticks) {

    *speed = (*dir_k) * (ticks);

    setAngle(ticks);

    (*state) = (*state) + (*base_int) * (*speed);

    return *state;
}

const Matrix Integrator::operator<< (const Matrix &ticks) const {
    
    *speed = (*dir_k) * (ticks);

    setAngle(ticks);

    (*state) = (*state) + (*base_int) * (*speed);

    Matrix ret(*state);

    return ret;
}

void Integrator::setAngle(Matrix ticks) const {
    double vSin = getT();
    if(method == RUNGE_KUTTA) {
        vSin += offset(ticks);
    }
    double vCos = cos(vSin);
    vSin = sin(vSin);
    base_int->set(0, 0,  vCos);
    base_int->set(0, 1, -vSin);
    base_int->set(1, 0,  vSin);
    base_int->set(1, 1,  vCos);
}

double Integrator::offset(Matrix ticks) const {
    double acc = 0;
    for(int i=0; i<signals; i++){
        acc += ticks.get(i,0);
    }
    return acc*rk_offset;
};