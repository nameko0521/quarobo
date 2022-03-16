#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int32MultiArray.h"
//#include "../include/kinematics.hpp"
#include <math.h>
#include <iostream>
using namespace std;

#define PI 3.141592

typedef struct{
    double x;
    double y;
    double z;
} VECTOR_3D;

void ik(VECTOR_3D, double *);
void move_ik(VECTOR_3D &, VECTOR_3D, double *, ros::Publisher);
void move_ik(VECTOR_3D &, VECTOR_3D, VECTOR_3D &, VECTOR_3D,
        double *, double *, ros::Publisher publisher);
void move_ik(VECTOR_3D &, VECTOR_3D ,
        VECTOR_3D &, VECTOR_3D,
        VECTOR_3D &, VECTOR_3D,
        VECTOR_3D &, VECTOR_3D,
        double *, double *, double *, double *,
        ros::Publisher);
void pub(ros::Publisher, double *);
void pub(ros::Publisher, double *, double *);
void pub(ros::Publisher, double *, double *, double *, double *);

// r = 54 + 68 = 122
double L1 = 54.00;
double L2 = 68.00;

double sx = 14.55 + 5.5;
double sy = 22.304;
double sz = 12.3;

int main(int argc, char **argv){

    ros::init(argc, argv, "jointStates_publisher");
    ros::NodeHandle nh;

    // position: 0 0 -122
    VECTOR_3D     current_position = {0, 0, -122};
    VECTOR_3D current_position_sub = {0, 0, -122};

    VECTOR_3D current_position_fr = {0, 0, -122};
    VECTOR_3D current_position_fl = {0, 0, -122};
    VECTOR_3D current_position_rr = {0, 0, -122};
    VECTOR_3D current_position_rl = {0, 0, -122};

    VECTOR_3D start_position = {10, 0, -110};

    VECTOR_3D crawl_position_fr = {10, 0, -100};
    VECTOR_3D crawl_position_fl = {10, 0, -110};
    VECTOR_3D crawl_position_rr = {10, 0, -110};
    VECTOR_3D crawl_position_rl = {10, 0, -110};

    VECTOR_3D crawl_motion_fr[10] = {
        {10,   0, -100},
        {10, -10, -110},
        {10, -10, -110},
        {10, -10, -110},
        {10, -10, -110},
        {10,   0, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
    };
    VECTOR_3D crawl_motion_rl[10] = {
        {10,   0, -110},
        {10,  10, -110},
        {10,   0, -100},
        {10, -10, -110},
        {10, -10, -110},
        {10,   0, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
    };
    VECTOR_3D crawl_motion_fl[10] = {
        {10,   0, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,   0, -100},
        {10,   0, -100},
        {10, -10, -110},
        {10, -10, -110},
        {10, -10, -110},
        {10, -10, -110},
    };
    VECTOR_3D crawl_motion_rr[10] = {
        {10,   0, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  10, -110},
        {10,  20, -110},
        {10,  30, -110},
        {10,  10, -100},
        {10,   0, -100},
        {10, -10, -110},
    };

    double servo_angle_fr[3] = {0.00, 0.00, 0.00};
    double servo_angle_fl[3] = {0.00, 0.00, 0.00};
    double servo_angle_rr[3] = {0.00, 0.00, 0.00};
    double servo_angle_rl[3] = {0.00, 0.00, 0.00};


    VECTOR_3D trot_position_a = {10, 10, -110};
    VECTOR_3D trot_position_b = {10,  0, -110};
    VECTOR_3D trot_motion_a[4] = {
        {10,   0, -100},
        {10, -10, -110},
        {10,   0, -110},
        {10,  10, -110},
    };
    VECTOR_3D trot_motion_b[4] = {
        {10,   0, -110},
        {10,  10, -110},
        {10,   0, -100},
        {10, -10, -110},
    };

    double servo_angle[3] = {0.00, 0.00, 0.00};
    double servo_angle_sub[3] = {0.00, 0.00, 0.00};

    //VECTOR_3D trot_motion_1[3] = {
    //    {10,   0, -100},
    //    {10,   0, -110},
    //    {10,  10, -110},
    //};
    //VECTOR_3D trot_motion_2[3] = {
    //    {10,  10, -110},
    //    {10,   0, -100},
    //    {10,   0, -110},
    //};

    VECTOR_3D    up_target_position = {10, 20,  -90};
    VECTOR_3D  down_target_position = {10, 20, -110};
    VECTOR_3D input_target_position = {0, 0, 0};

    ros::Publisher jointState_publish = nh.advertise<sensor_msgs::JointState>("/quarobo/joint_states", 120);
    while(ros::ok()){
        char key;
        cout << "key: ";
        cin >> key;
        switch(key) {
            case 'c': // crawl
                    move_ik(current_position_fr, start_position, current_position_fl, start_position,
                            current_position_rr, start_position, current_position_rl, start_position,
                            servo_angle_fr, servo_angle_fl, servo_angle_rr, servo_angle_rl,
                            jointState_publish);
                    move_ik(current_position_fr, crawl_position_fr, current_position_fl, crawl_position_fl,
                            current_position_rr, crawl_position_rr, current_position_rl, crawl_position_rl,
                            servo_angle_fr, servo_angle_fl, servo_angle_rr, servo_angle_rl,
                            jointState_publish);
                    while(true) {
                        for(int i=0; i < sizeof crawl_motion_fr/sizeof crawl_motion_fr[0]; i++) {
                            move_ik(current_position_fr, crawl_motion_fr[i], current_position_fl, crawl_motion_fl[i],
                                    current_position_rr, crawl_motion_rr[i], current_position_rl, crawl_motion_rl[i],
                                    servo_angle_fr, servo_angle_fl, servo_angle_rr, servo_angle_rl,
                                    jointState_publish);
                        }
                    }
                break;
            case 't': // trot
                move_ik(current_position, trot_position_a, current_position_sub, trot_position_b, servo_angle, servo_angle_sub, jointState_publish);
                while(true){
                    for(int i=0; i < sizeof trot_motion_a/sizeof trot_motion_a[0]; i++) {
                        move_ik(current_position, trot_motion_a[i], current_position_sub, trot_motion_b[i], servo_angle, servo_angle_sub, jointState_publish);
                    }
                }
                break;
            case 'i':
                cout << current_position.x << ", " << current_position.y << ", " << current_position.z << endl;
                cout << "position: ";
                cin >> input_target_position.x >> input_target_position.y >> input_target_position.z;
                move_ik(current_position, input_target_position, servo_angle, jointState_publish);
                break;
            case 'k':
                move_ik(current_position, down_target_position, servo_angle, jointState_publish);
                break;
            case 'j':
                move_ik(current_position, up_target_position, servo_angle, jointState_publish);
                break;
            case 'q':
                continue;
        }
    }
    return 0;
}

void ik(VECTOR_3D p, double *theta) {
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

    cout << "=====================" << endl;
    cout << "a: " << theta[0] << " : " << theta[0] * 180 / PI << endl;
    cout << "b: " << theta[1] << " : " << theta[1] * 180 / PI << endl;
    cout << "y: " << theta[2] << " : " << theta[2] * 180 / PI << endl;

    cout << endl;

    cout << "point:" << endl;
    cout << "px: " << p.x + sx << endl;
    cout << "py: " << p.y + sy << endl;
    cout << "pz: " << p.z - sz << endl;
    cout << "=====================" << endl;

    cout << endl;
}

void move_ik(VECTOR_3D &current_position, VECTOR_3D target_position, double *angle, ros::Publisher publisher){
    bool flag_x = false;
    bool flag_y = false;
    bool flag_z = false;
    while(flag_x == false || flag_y == false || flag_z == false) {
        if (current_position.x != target_position.x){
            current_position.x < target_position.x ? ++current_position.x : --current_position.x;
        }else {
            flag_x = true;
        }
        if (current_position.y != target_position.y){
            current_position.y < target_position.y ? ++current_position.y : --current_position.y;
        }else {
            flag_y = true;
        }
        if (current_position.z != target_position.z){
            current_position.z < target_position.z ? ++current_position.z : --current_position.z;
        }else {
            flag_z = true;
        }
        ik(current_position, angle);
        pub(publisher, angle);
    }
}

void move_ik(VECTOR_3D &current_position, VECTOR_3D target_position, VECTOR_3D &current_position_sub, VECTOR_3D target_position_sub,
        double *angle, double *angle_sub, ros::Publisher publisher){
    bool flag_x = false;
    bool flag_y = false;
    bool flag_z = false;
    bool flag_x_sub = false;
    bool flag_y_sub = false;
    bool flag_z_sub = false;
    while(flag_x == false || flag_y == false || flag_z == false || flag_x_sub == false || flag_y_sub == false || flag_z_sub == false) {
        if (current_position.x != target_position.x){
            current_position.x < target_position.x ? ++current_position.x : --current_position.x;
        }else {
            flag_x = true;
        }
        if (current_position.y != target_position.y){
            current_position.y < target_position.y ? ++current_position.y : --current_position.y;
        }else {
            flag_y = true;
        }
        if (current_position.z != target_position.z){
            current_position.z < target_position.z ? ++current_position.z : --current_position.z;
        }else {
            flag_z = true;
        }
        if (current_position_sub.x != target_position_sub.x){
            current_position_sub.x < target_position_sub.x ? ++current_position_sub.x : --current_position_sub.x;
        }else {
            flag_x_sub = true;
        }
        if (current_position_sub.y != target_position_sub.y){
            current_position_sub.y < target_position_sub.y ? ++current_position_sub.y : --current_position_sub.y;
        }else {
            flag_y_sub = true;
        }
        if (current_position_sub.z != target_position_sub.z){
            current_position_sub.z < target_position_sub.z ? ++current_position_sub.z : --current_position_sub.z;
        }else {
            flag_z_sub = true;
        }
        cout << "main" << endl;
        ik(current_position, angle);
        cout << "sub" << endl;
        ik(current_position_sub, angle_sub);
        pub(publisher, angle, angle_sub);
    }
}

void move_ik(VECTOR_3D &current_position_fr, VECTOR_3D target_position_fr,
        VECTOR_3D &current_position_fl, VECTOR_3D target_position_fl,
        VECTOR_3D &current_position_rr, VECTOR_3D target_position_rr,
        VECTOR_3D &current_position_rl, VECTOR_3D target_position_rl,
        double *angle_fr, double *angle_fl, double *angle_rr, double *angle_rl,
        ros::Publisher publisher){
    bool flag_x_fr = false;
    bool flag_y_fr = false;
    bool flag_z_fr = false;
    bool flag_x_fl = false;
    bool flag_y_fl = false;
    bool flag_z_fl = false;
    bool flag_x_rr = false;
    bool flag_y_rr = false;
    bool flag_z_rr = false;
    bool flag_x_rl = false;
    bool flag_y_rl = false;
    bool flag_z_rl = false;
    while(flag_x_fr == false || flag_y_fr == false || flag_z_fr == false ||
            flag_x_fl == false || flag_y_fl == false || flag_z_fl == false ||
            flag_x_rr == false || flag_y_rr == false || flag_z_rr == false ||
            flag_x_rl == false || flag_y_rl == false || flag_z_rl == false) {
        if (current_position_fr.x != target_position_fr.x){
            current_position_fr.x < target_position_fr.x ? ++current_position_fr.x : --current_position_fr.x;
        }else {
            flag_x_fr = true;
        }
        if (current_position_fr.y != target_position_fr.y){
            current_position_fr.y < target_position_fr.y ? ++current_position_fr.y : --current_position_fr.y;
        }else {
            flag_y_fr = true;
        }
        if (current_position_fr.z != target_position_fr.z){
            current_position_fr.z < target_position_fr.z ? ++current_position_fr.z : --current_position_fr.z;
        }else {
            flag_z_fr = true;
        }
        if (current_position_fl.x != target_position_fl.x){
            current_position_fl.x < target_position_fl.x ? ++current_position_fl.x : --current_position_fl.x;
        }else {
            flag_x_fl = true;
        }
        if (current_position_fl.y != target_position_fl.y){
            current_position_fl.y < target_position_fl.y ? ++current_position_fl.y : --current_position_fl.y;
        }else {
            flag_y_fl = true;
        }
        if (current_position_fl.z != target_position_fl.z){
            current_position_fl.z < target_position_fl.z ? ++current_position_fl.z : --current_position_fl.z;
        }else {
            flag_z_fl = true;
        }
        if (current_position_rr.x != target_position_rr.x){
            current_position_rr.x < target_position_rr.x ? ++current_position_rr.x : --current_position_rr.x;
        }else {
            flag_x_rr = true;
        }
        if (current_position_rr.y != target_position_rr.y){
            current_position_rr.y < target_position_rr.y ? ++current_position_rr.y : --current_position_rr.y;
        }else {
            flag_y_rr = true;
        }
        if (current_position_rr.z != target_position_rr.z){
            current_position_rr.z < target_position_rr.z ? ++current_position_rr.z : --current_position_rr.z;
        }else {
            flag_z_rr = true;
        }
        if (current_position_rl.x != target_position_rl.x){
            current_position_rl.x < target_position_rl.x ? ++current_position_rl.x : --current_position_rl.x;
        }else {
            flag_x_rl = true;
        }
        if (current_position_rl.y != target_position_rl.y){
            current_position_rl.y < target_position_rl.y ? ++current_position_rl.y : --current_position_rl.y;
        }else {
            flag_y_rl = true;
        }
        if (current_position_rl.z != target_position_rl.z){
            current_position_rl.z < target_position_rl.z ? ++current_position_rl.z : --current_position_rl.z;
        }else {
            flag_z_rl = true;
        }
        cout << "fr" << endl;
        ik(current_position_fr, angle_fr);
        cout << "fl" << endl;
        ik(current_position_fl, angle_fl);
        cout << "rr" << endl;
        ik(current_position_rr, angle_rr);
        cout << "rl" << endl;
        ik(current_position_rl, angle_rl);
        pub(publisher, angle_fr, angle_fl, angle_rr, angle_rl);
    }
}

void pub(ros::Publisher publisher, double *angle) {
    ros::Rate loop_rate(10);
    sensor_msgs::JointState quarobo_js;
    quarobo_js.header.stamp = ros::Time::now();
    quarobo_js.name.resize(12);
    quarobo_js.name[0]  = "FRS_joint";
    quarobo_js.name[1]  = "FRL_joint";
    quarobo_js.name[2]  = "FRF_joint";
    quarobo_js.name[3]  = "FLS_joint";
    quarobo_js.name[4]  = "FLL_joint";
    quarobo_js.name[5]  = "FLF_joint";
    quarobo_js.name[6]  = "RRS_joint";
    quarobo_js.name[7]  = "RRL_joint";
    quarobo_js.name[8]  = "RRF_joint";
    quarobo_js.name[9]  = "RLS_joint";
    quarobo_js.name[10] = "RLL_joint";
    quarobo_js.name[11] = "RLF_joint";
    quarobo_js.position.resize(12);
    quarobo_js.position[0]  =  angle[0];
    quarobo_js.position[1]  =  angle[1];
    quarobo_js.position[2]  =  angle[2];
    quarobo_js.position[3]  = -angle[0];
    quarobo_js.position[4]  =  angle[1];
    quarobo_js.position[5]  =  angle[2];
    quarobo_js.position[6]  =  angle[0];
    quarobo_js.position[7]  =  angle[1];
    quarobo_js.position[8]  =  angle[2];
    quarobo_js.position[9]  = -angle[0];
    quarobo_js.position[10] =  angle[1];
    quarobo_js.position[11] =  angle[2];
    publisher.publish(quarobo_js);
    loop_rate.sleep();
}

// trot
void pub(ros::Publisher publisher, double *angle, double *angle_sub) {
    ros::Rate loop_rate(30);
    sensor_msgs::JointState quarobo_js;
    quarobo_js.header.stamp = ros::Time::now();
    quarobo_js.name.resize(12);
    quarobo_js.name[0]  = "FRS_joint";
    quarobo_js.name[1]  = "FRL_joint";
    quarobo_js.name[2]  = "FRF_joint";
    quarobo_js.name[3]  = "FLS_joint";
    quarobo_js.name[4]  = "FLL_joint";
    quarobo_js.name[5]  = "FLF_joint";
    quarobo_js.name[6]  = "RRS_joint";
    quarobo_js.name[7]  = "RRL_joint";
    quarobo_js.name[8]  = "RRF_joint";
    quarobo_js.name[9]  = "RLS_joint";
    quarobo_js.name[10] = "RLL_joint";
    quarobo_js.name[11] = "RLF_joint";
    quarobo_js.position.resize(12);
    quarobo_js.position[0]  =  angle[0];
    quarobo_js.position[1]  =  angle[1];
    quarobo_js.position[2]  =  angle[2];
    quarobo_js.position[3]  = -angle_sub[0];
    quarobo_js.position[4]  =  angle_sub[1];
    quarobo_js.position[5]  =  angle_sub[2];
    quarobo_js.position[6]  =  angle_sub[0];
    quarobo_js.position[7]  =  angle_sub[1];
    quarobo_js.position[8]  =  angle_sub[2];
    quarobo_js.position[9]  = -angle[0];
    quarobo_js.position[10] =  angle[1];
    quarobo_js.position[11] =  angle[2];
    publisher.publish(quarobo_js);
    loop_rate.sleep();
}

// crawl
void pub(ros::Publisher publisher, double *angle_fr, double *angle_fl, double *angle_rr, double *angle_rl) {
    ros::Rate loop_rate(20);
    sensor_msgs::JointState quarobo_js;
    quarobo_js.header.stamp = ros::Time::now();
    quarobo_js.name.resize(12);
    quarobo_js.name[0]  = "FRS_joint";
    quarobo_js.name[1]  = "FRL_joint";
    quarobo_js.name[2]  = "FRF_joint";
    quarobo_js.name[3]  = "FLS_joint";
    quarobo_js.name[4]  = "FLL_joint";
    quarobo_js.name[5]  = "FLF_joint";
    quarobo_js.name[6]  = "RRS_joint";
    quarobo_js.name[7]  = "RRL_joint";
    quarobo_js.name[8]  = "RRF_joint";
    quarobo_js.name[9]  = "RLS_joint";
    quarobo_js.name[10] = "RLL_joint";
    quarobo_js.name[11] = "RLF_joint";
    quarobo_js.position.resize(12);
    quarobo_js.position[0]  =  angle_fr[0];
    quarobo_js.position[1]  =  angle_fr[1];
    quarobo_js.position[2]  =  angle_fr[2];
    quarobo_js.position[3]  = -angle_fl[0];
    quarobo_js.position[4]  =  angle_fl[1];
    quarobo_js.position[5]  =  angle_fl[2];
    quarobo_js.position[6]  =  angle_rr[0];
    quarobo_js.position[7]  =  angle_rr[1];
    quarobo_js.position[8]  =  angle_rr[2];
    quarobo_js.position[9]  = -angle_rl[0];
    quarobo_js.position[10] =  angle_rl[1];
    quarobo_js.position[11] =  angle_rl[2];
    publisher.publish(quarobo_js);
    loop_rate.sleep();
}
