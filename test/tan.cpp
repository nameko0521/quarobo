#include <iostream>
#include <math.h>
using namespace std;

#define PI 3.141592

int main() {
    double a = -atan2(2,-8);
    double b = a + PI;

    cout << a << endl;
    cout << b << endl;

    double L1 = link_parameter_3dof[0].length;
    double L2 = link_parameter_3dof[1].length;

    double dL1 = pow(L1, 2.0);
    double dL2 = pow(L2, 2.0);
    double dx_feet = pow(p.x, 2.0);
    double dy_feet = pow(p.y, 2.0);
    double dz_feet = pow(p.z, 2.0);

    double z_corr = -sqrt(dz_feet + dy_feet);
    double dz_corr = pow(z_corr, 2.0);
    double c = sqrt(dx_feet + dz_corr);
    double dc = pow(c, 2.0);

    double D1 = atan2(p.x, z_corr);
    double D2 = acos((dc + dL1 - dL2) / (2*c*L1));

    theta[0] = -atan2(p.y,p.z) + PI;
    theta[1] = D1 + D2;
    theta[2] = acos((dL1 + dL2 - dc) / (2*L1*L2)) - PI;

    return 0;
}
