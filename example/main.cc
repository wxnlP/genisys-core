#include "s6h4d_interface.h"
#include <iostream>
#include <eigen3/Eigen/Dense>


int main()
{
    Eigen::Matrix3d a;
    a << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

    std::cout << "A = \n" << a << std::endl;
    return 0;
}