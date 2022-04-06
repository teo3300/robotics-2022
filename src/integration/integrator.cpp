#include "integrator.hpp"
#include <cmath>

double zeroPose[3] = {0,0,0};
double set_base_int[9] = {0,0,0,0,0,0,0,0,1};

Integrator::Integrator(Method method){
    dir_k = nullptr;
    state = new Matrix(3,1,zeroPose);
    bufferState = new Matrix(3,1,zeroPose);
    base_int = new Matrix(3,3,set_base_int);
    counter = 0;
}
Integrator::~Integrator(){
    delete state;
    delete bufferState;
    delete base_int;
    delete dir_k;
}

void Integrator::dirKin(unsigned int h, unsigned int w, double mat[]) {
    dir_k = new Matrix(h, w, mat);
}

Matrix* Integrator::integrate(Matrix* ticks){
    setAngle(ticks);
    bufferState->mul(dir_k, ticks);
    bufferState->mul(base_int, bufferState);
    state->sum(state,bufferState);

    return state;
}

void Integrator::setAngle(Matrix* ticks) {
    double vSin = getT();
    if(method == EULER) {
        vSin += offset(ticks);
    }
    double vCos = cos(vSin);
    vSin = sin(vSin);
    base_int->set(0, 0,  vCos);
    base_int->set(0, 1, -vSin);
    base_int->set(1, 0,  vSin);
    base_int->set(1, 1,  vCos);
}

double Integrator::offset(Matrix* ticks) {
    double acc = 0;
    for(int i=0; i<WHEELS; i++){
        acc += ticks->get(i,0);
    }
    return acc*(RADIUS*PI_2/(T_ROUND*(LENGTH+WIDTH)));
};