#include <iostream>
#include <math.h>
#include "kinematics.hpp"

// r = 54 + 68 = 122
double L1 = 54.00;
double L2 = 68.00;

//double px = 10;
//double py = 20;
//double pz = -60.0;

double sx = 14.55 + 5.5;
double sy = 22.304;
double sz = 12.3;

int ik(VECTOR_3D p, double *theta) {
    double powL1 = pow(L1, 2.0);
    double powL2 = pow(L2, 2.0);
    double pow_px = pow(p.x, 2.0);
    double pow_py = pow(p.y, 2.0);
    double pow_pz = pow(p.z, 2.0);

    double z_corr = sqrt(pow_px + pow_pz);
    double pow_z_corr = pow(z_corr, 2.0);
    double c = sqrt(pow_py + pow_z_corr);
    double pow_c = pow(c, 2.0);

    double D1 = atan2(p.y, z_corr);
    double D2 = acos((pow_c + powL1 - powL2) / (2*c*L1));

    theta[0] = -atan2(p.x,p.z) + PI;
    theta[1] = D1 + D2;
    theta[2] = acos((powL1 + powL2 - pow_c) / (2*L1*L2)) - PI;

    //std::cout << std::endl;

    //std::cout <<  "D1: " << D1 << " : " << D1 * 180 / PI << std::endl;
    //std::cout <<  "D2: " << D2 << " : " << D2 * 180 / PI << std::endl;

    std::cout << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << "a: " << theta[0] << " : " << theta[0] * 180 / PI << std::endl;
    std::cout << "b: " << theta[1] << " : " << theta[1] * 180 / PI << std::endl;
    std::cout << "y: " << theta[2] << " : " << theta[2] * 180 / PI << std::endl;

    std::cout << std::endl;

    std::cout << "point:" << std::endl;
    std::cout << "px: " << p.x + sx << std::endl;
    std::cout << "py: " << p.y + sy << std::endl;
    std::cout << "pz: " << p.z - sz << std::endl;
    std::cout << "=====================" << std::endl;

    std::cout << std::endl;

    return 0;
}
