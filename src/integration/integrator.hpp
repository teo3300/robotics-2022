#ifndef INTEGRATOR_HPP_
#define INTEGRATOR_HPP_

#include "../geometry/matrix.hpp"

enum Method{EULER,RUNGE_KUTTA};


extern double zeroPose[3];
extern double set_base_int[9];

class Integrator {
    unsigned int counter;
    unsigned int signals;
    double period;
    Matrix* state;
    Matrix* base_int;
    Matrix* speed;
    Matrix* dir_k;
    Method method;

    inline void setX(double x) { (*state)(0, 0) = x; }
    inline void setY(double y) { (*state)(1, 0) = y; }
    inline void setT(double z) { (*state)(2, 0) = z; }
    inline double offset() { return getW() * period / 2; }
    void setAngle();
public:
    inline double getX() { return (*state)(0, 0); }
    inline double getY() { return (*state)(1, 0); }
    inline double getT() { return (*state)(2, 0); }
    inline double getVx() { return (*speed)(0, 0); }
    inline double getVy() { return (*speed)(1, 0); }
    inline double getW()  { return (*speed)(2, 0); }
    inline double getPeriod() { return period; }
    // Runge kutta integration
    Integrator(Method method, double period);
    ~Integrator();
    void dirKin(unsigned int h, unsigned int w, double mat[]);
    void inline setPos(double t[]) { state->fill(t); }
    void inline resetPos() { setPos(zeroPose); }
    Matrix integrate(Matrix speed);
    Matrix operator<< (Matrix speed);
};

inline Matrix operator>> (Matrix speed, Integrator& integrator) {
    return integrator << speed;
}

std::ostream& operator<<(std::ostream &strm, Integrator &mat);

#endif//INTEGRATOR_HPP_