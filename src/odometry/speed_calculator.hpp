#include "../geometry/matrix.hpp"

#include <math.h>

class SpeedCalculator {
    Matrix* speed;
    Matrix* kinematic;

public:
    SpeedCalculator(Matrix kinematic);
    ~SpeedCalculator();
    
    inline Matrix getSpeed() { return Matrix(*speed); }
    Matrix operator<< (Matrix input);
};

inline Matrix operator >> (Matrix speed, SpeedCalculator& speedCalculator) {
    return speedCalculator << speed;
}