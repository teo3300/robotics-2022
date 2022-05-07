#include "geometry/integrator.hpp"
#include <cmath>
#include <sstream>
#include <string>

double zeroPose[3] = {0,0,0};
double set_base_int[9] = {0,0,0,0,0,0,0,0,1};

Integrator::Integrator(Method method, double timeStamp)
        : method(method), currStamp(timeStamp), prevStamp(0) {
    state = new Matrix(3,1,zeroPose);
    base_int = new Matrix(3,3,set_base_int);
};

Integrator::Integrator(double timestamp) : Integrator(EULER, timestamp) {

};

Integrator::~Integrator(){
    delete state;
    delete base_int;
}

Matrix Integrator::operator<< (Matrix speed) {

    angularSpeed = speed(2,0);

    setAngle();

    // following theory: state + T_s * matrix * velocities
    (*state) = (*state) + getDt() * (*base_int) * (speed);

    Matrix ret(*state);

    return ret;
}

void Integrator::setAngle() {
    double vSin = getT();
    if(method == RUNGE_KUTTA) {
        vSin += offset();
    }
    double vCos = cos(vSin);
    vSin = sin(vSin);
    (*base_int)(0, 0) = vCos;
    (*base_int)(0, 1) = -vSin;
    (*base_int)(1, 0) = vSin;
    (*base_int)(1, 1) = vCos;
}