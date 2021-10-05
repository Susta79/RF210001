#ifndef ROBOT_H
#define ROBOT_H

#include "pose.h"
#include "joint.h"

enum Brand { IR, ABB, KUKA };

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
    Pose FK(Joint j);
    Eigen::Affine3d FK2(Joint j);
    Joint IK(Pose p, Joint jAct, bool bFrontBack, bool bUpDown);
};

#endif