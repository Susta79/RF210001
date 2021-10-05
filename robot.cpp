#include <iostream>
#include <iomanip>
#include "robot.h"
#include "pose.h"
#include "joint.h"

//#include <Eigen/Geometry>
#include <Eigen/Dense>
//#include <eigen_conversions/eigen_msg.h>
//#include <Eigen/Core>

Robot::Robot(void){
    this->a1z = 0.0;
    this->a2x = 0.0;
    this->a2z = 0.0;
    this->a3z = 0.0;
    this->a4x = 0.0;
    this->a4z = 0.0;
    this->a5x = 0.0;
    this->a6x = 0.0;
    this->brand = IR;
}

void Robot::setdimensions(
    double a1z,
    double a2x,
    double a2z,
    double a3z,
    double a4x,
    double a4z,
    double a5x,
    double a6x){
    this->a1z = a1z;
    this->a2x = a2x;
    this->a2z = a2z;
    this->a3z = a3z;
    this->a4x = a4x;
    this->a4z = a4z;
    this->a5x = a5x;
    this->a6x = a6x;
}

void Robot::setdimensionsIR(void){
    this->a1z = 650.0;
    this->a2x = 400.0;
    this->a2z = 680.0;
    this->a3z = 1100.0;
    this->a4x = 766.0;
    this->a4z = 230.0;
    this->a5x = 345.0;
    this->a6x = 244.0;
    this->brand = IR;
}

void Robot::setdimensionsABB(void){
    this->a1z = 0.0;
    this->a2x = 175.0;
    this->a2z = 495.0;
    this->a3z = 1095.0;
    this->a4x = 1230.5;
    this->a4z = 175.0;
    this->a5x = 0.0;
    this->a6x = 85.0;
    this->brand = ABB;
}

// j: joint values in radiants
// MP_to_tool0: transformation from MP (mounting point = center of axe 6) to tool0
Pose Robot::FK(Joint j){
    Pose VTR;
    Pose link1, link2, link3, link4, link5, link6;
    Matrix4d T16, T;
    Matrix3d Rarm, Rwrist, R16, R;

    // Link1: Base to Joint 1
    // Translation
    link1.setpos(0.0, 0.0, this->a1z);
    Eigen::Affine3d l1t(Eigen::Translation3d(Eigen::Vector3d(0, 0, this->a1z)));
    // Rotation around Z
    link1.setrot(0.0, 0.0, j.getj1());
    Eigen::Affine3d l1r(Eigen::AngleAxisd(j.getj1(), Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d l1 = l1t * l1r;

    std::cout << "link1.getR(): " << std::endl << link1.getR() << std::endl;
    std::cout << "Affine l1: " << std::endl << l1.matrix() << std::endl;

    // Link2: Joint 1 to Joint 2
    // Translation
    link2.setpos(this->a2x, 0.0, this->a2z);
    Eigen::Affine3d l2t(Eigen::Translation3d(Eigen::Vector3d(this->a2x, 0, this->a2z)));
    // Rotation around Y
    link2.setrot(0.0, j.getj2(), 0.0);
    Eigen::Affine3d l2r(Eigen::AngleAxisd(j.getj2(), Eigen::Vector3d::UnitY()));
    Eigen::Affine3d l2 = l2t * l2r;

    std::cout << "link2.getR(): " << std::endl << link2.getR() << std::endl;
    std::cout << "Affine l2: " << std::endl << l2.matrix() << std::endl;

    // Link3: Joint 2 to Joint 3
    // Translation
    link3.setpos(0.0, 0.0, this->a3z);
    Eigen::Affine3d l3t(Eigen::Translation3d(Eigen::Vector3d(0, 0, this->a3z)));
    // Rotation around Y
    link3.setrot(0.0, j.getj3(), 0.0);
    Eigen::Affine3d l3r(Eigen::AngleAxisd(j.getj3(), Eigen::Vector3d::UnitY()));
    Eigen::Affine3d l3 = l3t * l3r;

    std::cout << "link3.getR(): " << std::endl << link3.getR() << std::endl;
    std::cout << "Affine l3: " << std::endl << l3.matrix() << std::endl;

    // Link4: Joint 3 to Joint 4
    // Translation
    link4.setpos(this->a4x, 0.0, this->a4z);
    Eigen::Affine3d l4t(Eigen::Translation3d(Eigen::Vector3d(this->a4x, 0, this->a4z)));
    // Rotation around X
    link4.setrot(j.getj4(), 0.0, 0.0);
    Eigen::Affine3d l4r(Eigen::AngleAxisd(j.getj4(), Eigen::Vector3d::UnitX()));
    Eigen::Affine3d l4 = l4t * l4r;

    std::cout << "link4.getR(): " << std::endl << link4.getR() << std::endl;
    std::cout << "Affine l4: " << std::endl << l4.matrix() << std::endl;

    // Link5: Joint 4 to Joint 5
    // Translation
    link5.setpos(this->a5x, 0, 0);
    Eigen::Affine3d l5t(Eigen::Translation3d(Eigen::Vector3d(this->a5x, 0, 0)));
    // Rotation around Y
    link5.setrot(0.0, j.getj5(), 0.0);
    Eigen::Affine3d l5r(Eigen::AngleAxisd(j.getj5(), Eigen::Vector3d::UnitY()));
    Eigen::Affine3d l5 = l5t * l5r;

    std::cout << "link5.getR(): " << std::endl << link5.getR() << std::endl;
    std::cout << "Affine l5: " << std::endl << l5.matrix() << std::endl;

    // Link6: Joint 5 to Joint 6
    // Translation
    link6.setpos(this->a6x, 0, 0);
    Eigen::Affine3d l6t(Eigen::Translation3d(Eigen::Vector3d(this->a6x, 0, 0)));
    // Rotation around X
    link6.setrot(j.getj6(), 0.0, 0.0);
    Eigen::Affine3d l6r(Eigen::AngleAxisd(j.getj6(), Eigen::Vector3d::UnitX()));
    Eigen::Affine3d l6 = l6t * l6r;

    std::cout << "link6.getR(): " << std::endl << link6.getR() << std::endl;
    std::cout << "Affine l6: " << std::endl << l6.matrix() << std::endl;

    //T16 = T1*T2*T3*T4*T5*T6;
    T16 = link1.getT() * link2.getT() * link3.getT() * link4.getT() * link5.getT() * link6.getT();
    std::cout << "T16:\n" << T16 << std::endl;

    Eigen::Affine3d l16 = l1 * l2 * l3 * l4 * l5 * l6;
    std::cout << "l16:\n" << l16.matrix() << std::endl;

    //Matrix4d Ttool0;
    //Ttool0 = XYZABCtoT(tool0);
    //T = T16 * MP_to_tool0.getT();
    T = T16;

    Vector3d XYZ = T.topRightCorner(3,1);
    std::cout << "XYZ:\n" << XYZ << std::endl;
    std::cout << "l16.translation():\n" << l16.translation() << std::endl;

    //Rarm = R1*R2*R3;
    //Rarm = link1.getR() * link2.getR() * link3.getR();
    //Rwrist = R4*R5*R6;
    //Rwrist = link4.getR() * link5.getR() * link6.getR();
    //R16 = Rarm * Rwrist;
    R16 = T.topLeftCorner(3,3);
    std::cout << "R16:\n" << R16 << std::endl;
    std::cout << "l16.rotation():\n" << l16.rotation() << std::endl;

    Pose Rtmp;
    switch(this->brand)
    {
        case IR:
            std::cout << "FK IR\n";
            R = R16;
            break;
        case ABB:
            std::cout << "FK ABB\n";
            // Rotation 90° around Y
            Rtmp.setrot(0.0, M_PI/2.0, 0.0);
            R = R16 * Rtmp.getR();
            break;
        case KUKA:
            std::cout << "FK KUKA\n";
            R = R16;
            break;
    }

    VTR.setpos(XYZ);
    VTR.setrot(R);

    return VTR;
}

Eigen::Affine3d Robot::FK2(Joint j){
    // Link1: Base to Joint 1
    // Translation
    Eigen::Affine3d l1t(Eigen::Translation3d(Eigen::Vector3d(0, 0, this->a1z)));
    // Rotation around Z
    Eigen::Affine3d l1r(Eigen::AngleAxisd(j.getj1(), Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d l1 = l1t * l1r;

    // Link2: Joint 1 to Joint 2
    // Translation
    Eigen::Affine3d l2t(Eigen::Translation3d(Eigen::Vector3d(this->a2x, 0, this->a2z)));
    // Rotation around Y
    Eigen::Affine3d l2r(Eigen::AngleAxisd(j.getj2(), Eigen::Vector3d::UnitY()));
    Eigen::Affine3d l2 = l2t * l2r;

    // Link3: Joint 2 to Joint 3
    // Translation
    Eigen::Affine3d l3t(Eigen::Translation3d(Eigen::Vector3d(0, 0, this->a3z)));
    // Rotation around Y
    Eigen::Affine3d l3r(Eigen::AngleAxisd(j.getj3(), Eigen::Vector3d::UnitY()));
    Eigen::Affine3d l3 = l3t * l3r;

    // Link4: Joint 3 to Joint 4
    // Translation
    Eigen::Affine3d l4t(Eigen::Translation3d(Eigen::Vector3d(this->a4x, 0, this->a4z)));
    // Rotation around X
    Eigen::Affine3d l4r(Eigen::AngleAxisd(j.getj4(), Eigen::Vector3d::UnitX()));
    Eigen::Affine3d l4 = l4t * l4r;

    // Link5: Joint 4 to Joint 5
    // Translation
    Eigen::Affine3d l5t(Eigen::Translation3d(Eigen::Vector3d(this->a5x, 0, 0)));
    // Rotation around Y
    Eigen::Affine3d l5r(Eigen::AngleAxisd(j.getj5(), Eigen::Vector3d::UnitY()));
    Eigen::Affine3d l5 = l5t * l5r;

    // Link6: Joint 5 to Joint 6
    // Translation
    Eigen::Affine3d l6t(Eigen::Translation3d(Eigen::Vector3d(this->a6x, 0, 0)));
    // Rotation around X
    Eigen::Affine3d l6r(Eigen::AngleAxisd(j.getj6(), Eigen::Vector3d::UnitX()));
    Eigen::Affine3d l6 = l6t * l6r;

    //T16 = T1*T2*T3*T4*T5*T6;
    Eigen::Affine3d l16 = l1 * l2 * l3 * l4 * l5 * l6;

    switch(this->brand)
    {
        case IR:
            std::cout << "FK IR\n";
            break;
        case ABB:
            std::cout << "FK ABB\n";
            // Rotation 90° around Y
            l16.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
            break;
        case KUKA:
            std::cout << "FK KUKA\n";
            break;
    }

    std::cout << "l16.translation():\n" << l16.translation() << std::endl;
    std::cout << "l16.rotation():\n" << l16.rotation() << std::endl;

    return l16;
}

Joint Robot::IK(Pose p, Joint jAct, bool bFrontBack, bool bUpDown){
    double J1, J2, J3, J4, J5, J6;
    double Rwrist11, Rwrist21, Rwrist31, Rwrist12, Rwrist13, Rwrist32, Rwrist33;
    double WPxy, l, h;
    double rho, b4x;
    double alpha, beta, gamma, delta;
    double cos_beta, sin_beta;
    Vector3d x_hat, WP;
    //Pose of the mounting point (center of the axe 6)
    Pose MP;
    Pose Rtmp;

    switch(this->brand)
    {
        case IR:
            std::cout << "IK IR\n";
            MP = p;
            break;
        case ABB:
            std::cout << "IK ABB\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.setpos(p.getpos());
            // Rotation 90° around Y
            Rtmp.setrot(0.0, M_PI/2.0, 0.0);
            MP.setrot(p.getR() * Rtmp.getR().transpose());
            break;
        case KUKA:
            std::cout << "IK KUKA\n";
            MP = p;
            break;
    }

    x_hat = MP.getR() * ((Vector3d() << 1, 0, 0).finished());
    WP = MP.getpos() - (this->a6x * x_hat);

    // Find J1
    // Check if there is a shoulder singularity
    if((abs(WP(0)) < 0.001) && (abs(WP(1)) < 0.001)){
        // In this case we have a shoulder singularity.
        // Fix J1 as the actual value of J1
        J1 = jAct.getj1();
    }
    else{
        // FRONT solution
        J1 = atan2(WP(1), WP(0));
        // To have the BACK solution I need to add or substract pi
        if(bFrontBack){
            // BACK solution is selected
            if(J1 > 0)
                J1 -= M_PI;
            else
                J1 += M_PI;
        }
    }

    // Find J2 and J3
    WPxy = sqrt( pow(WP(0),2) + pow(WP(1),2) );
    //std::cout << "WPxy:\n" << WPxy << std::endl;

    if(bFrontBack){
        // BACK solution
        l = WPxy + this->a2x;
    }
    else{
        // FRONT solution
        l = WPxy - this->a2x;
    }
    //std::cout << "l:\n" << l << std::endl;
    h = WP(2) - this->a1z - this->a2z;
    //std::cout << "h:\n" << h << std::endl;

    rho = sqrt( pow(h,2) + pow(l,2) );
    //std::cout << "rho:\n" << rho << std::endl;
    b4x = sqrt( pow(this->a4z,2) + pow(this->a4x+this->a5x,2) );
    //std::cout << "b4x:\n" << b4x << std::endl;
    if(rho >= (this->a3z+b4x)){
        // It is not possible to reach that point
        std::cout << "Error: impossible to reach that point" << std::endl;
        return jAct;
    }
    if(rho <= abs(a3z-b4x)){
        // J2 too close to the robot himself
        std::cout << "Error: J2 too small" << std::endl;
        return jAct;
    }

    alpha = atan2(h, l);
    cos_beta = (pow(rho,2) + pow(a3z,2) - pow(b4x,2)) / (2*rho*a3z);
    sin_beta = sqrt(1 - pow(cos_beta,2));
    beta = atan2(sin_beta, cos_beta);

    if(bUpDown){
        // DOWN solution
        J2 = M_PI_2 - alpha + beta;
    }     
    else{
        // UP solution
        J2 = M_PI_2 - alpha - beta;
    }

    double cos_gamma, sin_gamma;
    cos_gamma = (pow(a3z,2) + pow(b4x,2) - pow(rho,2)) / (2*a3z*b4x);
    sin_gamma = sqrt(1 - pow(cos_gamma,2));
    gamma = atan2(sin_gamma, cos_gamma);

    delta = atan2(this->a4x+this->a5x, this->a4z);

    J3 = M_PI - gamma - delta;

    //std::cout << "J1:\n" << J1 << std::endl;
    //std::cout << "J2:\n" << J2 << std::endl;
    //std::cout << "J3:\n" << J3 << std::endl;

    Pose pJ1, pJ23;
    pJ1.setrot(0.0, 0.0, J1);
    pJ23.setrot(0.0, (J2+J3), 0.0);
    //Matrix3d Rarm = RaroundZ(J1) * RaroundY(J2+J3);
    Matrix3d Rarm = pJ1.getR() * pJ23.getR();
    Matrix3d Rwrist = Rarm.transpose() * MP.getR();

    //Find J4, J5, J6

    Rwrist11 = Rwrist(0,0);
    Rwrist21 = Rwrist(1,0);
    Rwrist31 = Rwrist(2,0);
    Rwrist12 = Rwrist(0,1);
    Rwrist13 = Rwrist(0,2);
    Rwrist32 = Rwrist(2,1);
    Rwrist33 = Rwrist(2,2);

    if (Rwrist11 < 0.9999999) {
        if (Rwrist11 > -0.9999999) {
            //J5 = acos(Rwrist11);
            J5 = atan2( sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
            J4 = atan2(Rwrist21,-Rwrist31);
            J6 = atan2(Rwrist12,Rwrist13);
        }
        else // Rwrist11 = −1 
        {
            std::cout << "Rwrist11 = -1" << std::endl;
            // Wrist singularity. J5 = 180 -> This condition is not
            // possible because the spherical wrist cannot rotate J5 = 180.
            // Not a unique solution: J6 − J4 = atan2(Rwrist32,Rwrist33)
            J5 = M_PI;
            J4 = jAct.getj4();
            J6 = atan2(Rwrist32,Rwrist33) + J4;
        }
    }
    else // Rwrist11 = +1
    {
        std::cout << "Rwrist11 = +1" << std::endl;
        // Wrist singularity. J5 = 0
        // Not a unique solution: J4 + J6 = atan2(Rwrist32,Rwrist33)
        J5 = 0;
        J4 = jAct.getj4();
        J6 = atan2(Rwrist32,Rwrist33) - J4;
    }

    Joint j(J1, J2, J3, J4, J5, J6);
    return j;
}