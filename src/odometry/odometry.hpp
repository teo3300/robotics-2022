#include "../geometry/matrix.hpp"

#include <math.h>

class Odometry {
    Matrix* kinematic;

public:
    Odometry(Matrix kinematic);
    ~Odometry();
    Matrix operator<< (Matrix input);
};
