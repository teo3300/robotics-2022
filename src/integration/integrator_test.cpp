#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"
#include "../integration/integrator.hpp"

#include <iostream> 

double STAND[] ={0,0,0,0};
double ROT[] = {1,1,1,1};
double FWD[] = {-1,1,-1,1};

int main(){
    Integrator I(RUNGE_KUTTA,
                    (RADIUS*PI_2/(T_ROUND*(LENGTH+WIDTH))));
    I.dirKin(3, 4, dir_kin_mat);
    Matrix stand(4,1,STAND);
    Matrix fwd(4,1,FWD);
    Matrix rot(4,1,ROT);
    Matrix* output;
    output = I.integrate(&stand);
    std::cout << "Matrice stato dopo movimento nullo:\n" << *output << "\n";
    output = I.integrate(&rot);
    std::cout << "Matrice stato dopo rotazione:\n" << *output << "\n";
    output = I.integrate(&fwd);
    std::cout << "Matrice stato dopo avanzamento:\n" << *output << "\n";
}