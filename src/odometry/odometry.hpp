#include "../geometry/matrix.hpp"

#include <math.h>

class Odometry {
    Matrix* speed;
    Matrix* kinematic;

public:
    Odometry(Matrix kinematic);
    ~Odometry();
    void resetKinematic(Matrix newKinematic);
    Matrix getSpeed();
    Matrix operator<< (Matrix input);
};

inline Matrix operator >> (Matrix speed, Odometry& odometry) {
    return odometry << speed;
}