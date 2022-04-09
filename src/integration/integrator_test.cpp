#include "../constants/constants.hpp"
#include "../geometry/matrix.hpp"
#include "../integration/integrator.hpp"
#include "../odometry/odometry.hpp"

#include <iostream> 

double STAND[] ={0,0,0,0};
double ROT[] = {100,100,100,100};
double FWD[] = {-100,100,-100,100};
double FR[] = {-90,110,-90,110};

int main(){
    Integrator I(RUNGE_KUTTA, PERIOD);

    Odometry O(Matrix(3,4,dis_dir_kin_mat));
    
    Matrix stand(4,1,STAND);
    Matrix fwd(4,1,FWD);
    Matrix rot(4,1,ROT);
    Matrix fr(4,1,FR);
    
    Matrix output(3,1);

    output = stand >> O >> I;
    std::cout << "Stato nullo:\n" << output << "\n";
    output = rot >> O >> I;
    std::cout << "Stato rotazione:\n" << output << "\n";
    output = fwd >> O >> I;
    std::cout << "Stato avanzamento:\n" << output << "\n";
    output = fr >> O >> I;
    std::cout << "Stato roto-avanzamento:\n" << output << "\n";
}