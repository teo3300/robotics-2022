#include "odometry.hpp"
#include "../constants/constants.hpp"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdlib.h>

int main(int argc, char *argv[]){
    if(argc != 5) {
        std::cout << "Bad args\n";
    }
    Matrix* matrice = new Matrix(3,4,dir_kin_mat);
    Odometry angular(matrice);
    double swap[WHEELS];
    for(int i=0;i<WHEELS;i++){
        swap[i]=atof(argv[i+1]);
    }
    angular.setVal(swap);
    angular.compute();
    Matrix* A = angular.getResults();
    std::cout << "Matrice A w : "<<"\n"<<(*A)<<"\n";
    delete matrice;
    Matrix* matrice1 = new Matrix(3,4,dis_dir_kin_mat);
    angular.setModel(matrice1);
    angular.compute();
    A = angular.getResults();
    std::cout << "Matrice A ticks: "<<"\n"<<(*A)<<"\n";
}