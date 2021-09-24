/*
DoF: 3
LINK: 2
JOINT: 3

L1,  L2  -> link length.
L1C, L2C -> link cos.
L1S, L2S -> link sin.
*/

#include <stdio.h>
#include <math.h>
#include "kinematics.hpp"
#include "leg_parameter.hpp"

static LINK_PARAM link_parameter_3dof[LINK_NUM] = {0};
//static JOINT_RANGE joint_range[JOINT_NUM] = {0};

void initParam(void){
    getLinkParam3dof(link_parameter_3dof);
}

int forward_Kinematics3dof(VECTOR_3D *p, double *theta){

    double L1 = link_parameter_3dof[0].length;
    double L2 = link_parameter_3dof[1].length;

    double S1 = sin(theta[0]);
    double C1 = cos(theta[0]);
    double S2 = sin(theta[1]);
    double C2 = cos(theta[1]);
    double S23 = sin(theta[1] + theta[2]);
    double C23 = cos(theta[1] + theta[2]);

    p->x = C1 * (L1 * C2 + L2 * C23);
    p->y = S1 * (L1 * C2 + L2 * C23);
    p->z = L1 * S2 + L2 * S23;

    return 0;
}

int inverse_Kinematics3dof(VECTOR_3D p, double *theta){

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
