/*
    DoF: 3
    LINK: 2
    JOINT: 3
*/
#include "leg_parameter.hpp"

static const LINK_PARAM link_parameter_3dof[LINK_NUM] = {
 //  len,  incp_x,incp_y,incp_z
    {0.054, 0.000, 22.30, -12.30}, // link 1
    {0.088, -15.55, 0.000, 0.000}, // link 2
};

void getLinkParam3dof(LINK_PARAM *link_parameter){
    for(int i=0; i < LINK_NUM; i++){
        link_parameter[i] = link_parameter_3dof[i];
    }
}
