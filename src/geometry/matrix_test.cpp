#include "matrix/matrix.hpp"
#include <iostream>

double testA[] = {
    1,2,3,4,
    3,2,1,4,
    1,2,3,4
};
double testB[] = {
    4,5,
    6,1,
    1,6,
    5,4
};
double testC[] = {
    1,2,
    3,2,
    1,2
};

int main(){
    Matrix a(3,4,testA);
    Matrix b(4,2,testB);
    Matrix c = a*b;
    Matrix d = c*2;
    std::cout << "Matrix A:\n" << a << "\n";
    std::cout << "Matrix B:\n" << b << "\n";
    std::cout << "Matrix C:\n" << c << "\n";
    std::cout << "Matrix D:\n" << d << "\n";
    Matrix e(4,1,testA);
    Matrix f(4,1,testA);
    Matrix g = e*2+f;
    std::cout << "Matrix E:\n" << e << "\n";
    std::cout << "Matrix F:\n" << f << "\n";
    std::cout << "Matrix G:\n" << g << "\n";
    Matrix h = !g;
    std::cout << "Matrix G^T:\n" << h << "\n";
}