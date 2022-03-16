#include <iostream>
#include <math.h>
using namespace std;

#define PI 3.141592

int forward_Kinematics3dof(double *theta);

// r = 54 + 68 = 122
double L1 = 54.00;
double L2 = 68.00;

double px = 10;
double py = 20;
double pz = -60.0;

double powL1 = pow(L1, 2.0);
double powL2 = pow(L2, 2.0);
double pow_px = pow(px, 2.0);
double pow_py = pow(py, 2.0);
double pow_pz = pow(pz, 2.0);

double z_corr = sqrt(pow_px + pow_pz);
double pow_z_corr = pow(z_corr, 2.0);
double c = sqrt(pow_py + pow_z_corr);
double pow_c = pow(c, 2.0);

double sx = 14.55 + 5.5;
double sy = 22.304;
double sz = 12.3;

int main() {
    double D1 = atan2(py, z_corr);
    double D2 = acos((pow_c + powL1 - powL2) / (2*c*L1));

    double a = -atan2(px,pz) + PI;
    double b = D1 + D2;
    double y = acos((powL1 + powL2 - pow_c) / (2*L1*L2)) - PI;

    cout << endl;

    cout <<  "D1: " << D1 << " : " << D1 * 180 / PI << endl;
    cout <<  "D2: " << D2 << " : " << D2 * 180 / PI << endl;

    cout << endl;

    cout << "a: " << a << " : " << a * 180 / PI << endl;
    cout << "b: " << b << " : " << b * 180 / PI << endl;
    cout << "y: " << y << " : " << y * 180 / PI << endl;

    cout << endl;
    
    cout << "point:" << endl;
    cout << "px: " << px + sx << endl;
    cout << "py: " << py + sy << endl;
    cout << "pz: " << pz - sz << endl;

    cout << endl;
    double theta[3] = {a, b, y};
    forward_Kinematics3dof(theta);

    return 0;
}

int forward_Kinematics3dof(double *theta){
    double sin_a = sin(theta[0]);
    double cos_a = cos(theta[0]);
    double sin_b = sin(theta[1]);
    double cos_b = cos(theta[1]);
    double sin_by = sin(theta[1] + theta[2]);
    double cos_by = cos(theta[1] + theta[2]);

    cout << "x: " << cos_a * (L1 * cos_b + L2 * cos_by) - sx << endl;
    cout << "y: " << sin_a * (L1 * cos_b + L2 * cos_by) - sy << endl;
    cout << "z: " << L1 * sin_b + L2 * sin_by           - sz << endl;

    double theta_yb = theta[2] - theta[1];
    double x = L1 * sin_b + L2 * sin(theta_yb);
    double y = sin_a * (L1*cos_b + L2*cos(theta_yb));
    double z = cos_a * (L1*cos_b + L2*cos(theta_yb));

    cout << "gx: " << x - sx<< endl;
    cout << "gy: " << y - sy<< endl;
    cout << "gz: " << z - sz << endl;

    return 0;
}
