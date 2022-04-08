#include "integrator.hpp"
#include <cmath>
#include <sstream>
#include <string>

double zeroPose[3] = {0,0,0};
double set_base_int[9] = {0,0,0,0,0,0,0,0,1};

Integrator::Integrator(Method method, double period)
        : method(method), dir_k(nullptr), counter(0), period(period) {
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

Matrix Integrator::integrate(Matrix speed) {

    *this->speed = speed;

    setAngle();

    (*state) = (*state) + (*base_int) * (speed);

    return *state;
}

Matrix Integrator::operator<< (Matrix speed) {

    *this->speed = speed;

    setAngle();

    (*state) = (*state) + (*base_int) * (speed);

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
    base_int->set(0, 0,  vCos);
    base_int->set(0, 1, -vSin);
    base_int->set(1, 0,  vSin);
    base_int->set(1, 1,  vCos);
}

std::ostream& operator<<(std::ostream &strm, Integrator &intg) {
    std::string buffer = "";
    buffer += "state\tveolcity\n";
    buffer += std::to_string(intg.getX()) + "\t" + std::to_string(intg.getVx()) + "\n";
    buffer += std::to_string(intg.getY()) + "\t" + std::to_string(intg.getVy()) + "\n";
    buffer += std::to_string(intg.getT()) + "\t" + std::to_string(intg.getW()) + "\n";
    return strm << buffer;
}