#include "odometry.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdlib.h>

int main(int argc, char *argv[]){
    if(argc != 5) {
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
    //Odometry ticks(TICKS);

}