#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"

#include <math.h>

enum Source{ANG_VEL,TICKS};

class Odometry {
    Source source;
    Matrix* state;
    Matrix* result;
public:
    Odometry(Source source);
    ~Odometry();
    //set value of entire array
    void setVal(double source[]){return state->fill(source);};
    Matrix* getResults(){return this->result;};
    void compute();
};