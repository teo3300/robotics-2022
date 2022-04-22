#include "proj_const/constants.hpp"

#include <string>

Parameters robot_parameters;

const double dir_kin_template[WHEELS * VARS] = {
     1,   1,   1,   1,
    -1,   1,   1,  -1,
    -1,   1,  -1,   1
};

const double inv_kin[VARS * WHEELS] = {
    1,  -1,  -1,
    1,   1,   1,
    1,   1,  -1,
    1,  -1,   1
};

bool load_parameters(int argc, char* argv[]) {

    // always pass 4 values, 0.0 to keep previous
    if(argc < SETTABLE+1) return false;

    // set specified arguments
    for(int i=0; i<SETTABLE; i++){
        robot_parameters.list[i] = std::stod(argv[i+1]);
    }

    return true;
}

void generateDirectKinematic(double* mem, double scalar) {
    for(int i=0; i<WHEELS*VARS; i++) {
        mem[i] = dir_kin_template[i]*scalar;
        if(i >= WHEELS * (VARS-1)) mem[i] = mem[i] / L;
    }
}
void generateInverseKinematic(double* mem, double scalar) {
    for(int i=0; i<WHEELS*VARS; i++) {
        mem[i] = dir_kin_template[i]*scalar;
        if(!((i+1) % VARS)) mem[i] = mem[i] / L;
    }
}