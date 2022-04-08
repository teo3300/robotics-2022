#include "../geometry/matrix.hpp"

#include <math.h>

class Odometry {
    Matrix* model;
    Matrix* state;
    Matrix* result;
    Matrix* kinematic;

public:
    Odometry(Matrix* model);
    ~Odometry();
    //set value for omega or ticks
    void setVal(double source[]){return state->fill(source);};
    void setModel(Matrix* m){this->model = m;};
    Matrix* getResults(){return this->result;};
    void compute();
};
