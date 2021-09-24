/*
    DoF: 3
    LINK: 2
    JOINT: 3
*/
#include <iostream>
#include "../include/leg_parameter.hpp"
#include "../include/kinematics.hpp"

using namespace std;

int main(int argc, char **argv){

    double start_angle[JOINT_NUM] = {0.30, 0.60, -1.00};
    VECTOR_3D goal_pose = {0};

    VECTOR_3D start_pose = {0.00, 0.125621 , -0.00377812};
    double goal_angle[JOINT_NUM] = {0};

    initParam();

    forward_Kinematics3dof(&goal_pose, start_angle);
    inverse_Kinematics3dof(goal_pose, goal_angle);

    cout << "Forward Kinematics result x: " << goal_pose.x << endl;
    cout << "Forward Kinematics result y: " << goal_pose.y << endl;
    cout << "Forward Kinematics result z: " << goal_pose.z << endl;
    cout << endl;
    cout << "Inverse Kinematics result  Shoulder: " << goal_angle[0] << endl;
    cout << "Inverse Kinematics result       Leg: " << goal_angle[1] << endl;
    cout << "Inverse Kinematics result      Foot: " << goal_angle[2] << endl;

    return 0;
}
