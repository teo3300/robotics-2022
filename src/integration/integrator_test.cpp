#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"
#include "../integration/integrator.hpp"

#include <iostream> 

double STAND[] ={0,0,0,0};
double ROT[] = {2,2,2,2};
double FWD[] = {-2,2,-2,2};

int main(){
    Integrator I(EULER);
    I.dirKin(3, 4, dir_kin_mat);
    Matrix stand(4,1,STAND);
    Matrix fwd(4,1,FWD);
    Matrix rot(4,1,ROT);
    Matrix* output;
    output = I.integrate(&stand);
    std::cout << "Matrice posizione dopo movimento nullo:\n" << *output << "\n";
    output = I.integrate(&rot);
    std::cout << "Matrice posizione dopo rotazione:\n" << *output << "\n";
    output = I.integrate(&fwd);
    std::cout << "Matrice posizione dopo avanzamento:\n" << *output << "\n";
}