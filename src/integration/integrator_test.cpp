#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"
#include "../integration/integrator.hpp"

#include <iostream> 

double STAND[] ={0,0,0,0};
double ROT[] = {100,100,100,100};
double FWD[] = {-100,100,-100,100};
double FR[] = {0,100,0,100};

int main(){
    Integrator I(EULER, RUNGE_KUTTA_OFFSET);
    I.dirKin(3, 4, dis_dir_kin_mat);
    Matrix stand(4,1,STAND);
    Matrix fwd(4,1,FWD);
    Matrix rot(4,1,ROT);
    Matrix fr(4,1,FR);
    Matrix output(3,1);
    output = I << stand;
    std::cout << "Stato nullo:\n" << output << "\n";
    output = I << rot;
    std::cout << "Stato rotazione:\n" << output << "\n";
    output = I << fwd;
    std::cout << "Stato avanzamento:\n" << output << "\n";
    output = I << fr;
    std::cout << "Stato roto-avanzamento:\n" << output << "\n";
    Matrix culo = Matrix(3,4,dis_dir_kin_mat)*rot;
    std::cout << "culo:\n" << culo << "\n";
}