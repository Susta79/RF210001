#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include "joint.h"

using namespace std;
using namespace Eigen;

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
    string model;
    string brand_string();
public:
    Robot();
    Robot(Brand b, string model);
    void setdimensions(double a1z, double a2x, double a2z, double a3z, double a4x, double a4z, double a5x, double a6x);
    void setdimensionsIR(void);
    void setdimensionsABB(void);
    void setdimensionsKUKA(void);
    void printAffine3d(Affine3d p);
    Affine3d FK(Joint j, Affine3d UT, Affine3d UF);
    Joint IK(Affine3d p, Affine3d UT, Affine3d UF, Joint jAct, FrontBack FB, UpDown UD, PositiveNegative PN);
};

#endif