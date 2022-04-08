#include "geometry/matrix.hpp"
#include "odometry.hpp"
#include "../constants/constants.hpp"

#include <iostream>

int main(int argc, char *argv[]){
    if(argc != 5) {
        std::cout << "Bad args\n";
        return 1;
    }
    

    double swap[WHEELS];
    for(int i=0;i<WHEELS;i++){
        swap[i]=atof(argv[i+1]);
    }
    Matrix input(4,1,swap);

    Odometry velocity(Matrix(3,4,dir_kin_mat));
    Matrix speed = velocity << input;
    std::cout << "Matrice A w : "<<"\n"<< speed <<"\n";

    Odometry ticks(Matrix(3,4,dis_dir_kin_mat));
    speed = ticks << input;
    std::cout << "Matrice A ticks: "<<"\n"<< speed <<"\n";
    
    return 0;
}