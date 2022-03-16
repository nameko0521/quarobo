#ifdef _KINEMATICS_
#define _KINEMATICS_

#define PI 3.141592

typedef struct{
    double x;
    double y;
    double z;
} VECTOR_3D;

int ik(VECTOR_3D, double *);
#endif
