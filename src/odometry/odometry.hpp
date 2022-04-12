#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "integrator.hpp"
#include "speed_calculator.hpp"
#include "../matrix/matrix.hpp"

class Odometry {
    Integrator* integrator;
    SpeedCalculator* speedCalculator;
public:
    Odometry(SpeedCalculator& speedCalculator, Integrator& integrator);
    ~Odometry();
    inline Matrix getSpeed() { return speedCalculator->getSpeed(); }
    inline Matrix getPosition() { return integrator->getPosition(); }
    Matrix operator<< (Matrix inputs);
};

#endif//ODOMETRY_HPP_