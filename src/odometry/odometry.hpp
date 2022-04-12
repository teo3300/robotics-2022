#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "../odometry/integrator.hpp"
#include "../odometry/speed_calculator.hpp"
#include "../geometry/matrix.hpp"

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