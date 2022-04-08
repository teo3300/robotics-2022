#include "../geometry/matrix.hpp"
#include "odometry.hpp"
#include "../constants/constants.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdlib.h>

int main(int argc, char *argv[]){
    /*if(argc != 5) {
        std::cout << "Bad args\n";
    }
    Odometry angular(ANG_VEL);
    double swap[WHEELS];
    for(int i=0;i<WHEELS;i++){
        swap[i]=atof(argv[i+1]);
    }
    angular.setVal(swap);
    angular.compute();
    Matrix* A = angular.getResults();
    std::cout << "Matrice A : "<<"\n"<<(*A)<<"\n";
    //Odometry ticks(TICKS);*/

    double fwd[] = {100,100,100,100};

    Matrix ticks(4,1,fwd);

    Odometry test(3,4,dis_dir_kin_mat);
    Matrix result = test << ticks;
    std::cout << "culoo\n" << result << "\n";

}