#include "geometry/matrix.hpp"

#include <math.h>

typedef enum { DISCRETE=false, CONTINUE=true } ComputeType;

class SpeedCalculator {
    ComputeType computeType;
    double currStamp;
    double prevStamp;
    Matrix* speed;
    Matrix* kinematic;

    inline double getDt() { return currStamp - prevStamp; }

public:
    SpeedCalculator(ComputeType computeType);
    ~SpeedCalculator();
    
    inline Matrix getSpeed() { return Matrix(*speed); }

    void setKinematic (Matrix kinematic);

    // needed since working with discrete values
    inline SpeedCalculator& setTimeStamp(double newStamp) {
        prevStamp = currStamp;
        currStamp = newStamp;
        return *this;
    }

    Matrix operator<< (Matrix input);
};

inline Matrix operator >> (Matrix speed, SpeedCalculator& speedCalculator) {
    return speedCalculator << speed;
}