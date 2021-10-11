#ifndef ROBOT_H
#define ROBOT_H

#include "joint.h"

enum Brand { IR, ABB, KUKA };
enum FrontBack { Front, Back };
enum UpDown { Up, Down };
enum PositiveNegative { Positive, Negative };

class Robot {
private:
    double a1z;
    double a2x;
    double a2z;
    double a3z;
    double a4x;
    double a4z;
    double a5x;
    double a6x;
    Brand brand;
public:
    Robot();
    void setdimensions(double a1z, double a2x, double a2z, double a3z, double a4x, double a4z, double a5x, double a6x);
    void setdimensionsIR(void);
    void setdimensionsABB(void);
    void setdimensionsKUKA(void);
    void printAffine3d(Eigen::Affine3d p);
    Eigen::Affine3d FK(Joint j, Eigen::Affine3d UT, Eigen::Affine3d UF);
    Joint IK(Eigen::Affine3d p, Eigen::Affine3d UT, Eigen::Affine3d UF, Joint jAct, FrontBack FB, UpDown UD, PositiveNegative PN);
};

#endif