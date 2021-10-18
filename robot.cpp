#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <Eigen/Dense>
#include <filesystem>
#include "robot.h"
#include "joint.h"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

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
    this->model = "";
}

string Robot::brand_string(){
    switch (this->brand)
    {
    case ABB:
        return "ABB";
    case KUKA:
        return "KUKA";
    case IR:
        return "IR";
    default:
        return "";
        break;
    }
}

Robot::Robot(Brand b, string m){
    this->brand = b;
    this->model = m;
    string filename = this->brand_string() + "_" + this->model + ".cfg";
    fs::path pathCfgFolder = fs::current_path().concat("/../cfg/");
    fs::path pathCfgFile = pathCfgFolder.concat(filename);
    ifstream infile(pathCfgFile.c_str());
    string line, par_name, par_val;
    cout << "Loading Robot from file: " << filename << "...";
    if(infile.is_open()){
        while (getline(infile, line)){
            size_t f = line.find_first_of(" ");
            par_name = line.substr(0,f);
            par_val = line.substr(f+1,line.size()-f);
            if (par_name == "a1z")
                this->a1z = stod(par_val);
            else if (par_name == "a2x")
                this->a2x = stod(par_val);
            else if (par_name == "a2z")
                this->a2z = stod(par_val);
            else if (par_name == "a3z")
                this->a3z = stod(par_val);
            else if (par_name == "a4x")
                this->a4x = stod(par_val);
            else if (par_name == "a4z")
                this->a4z = stod(par_val);
            else if (par_name == "a5x")
                this->a5x = stod(par_val);
            else if (par_name == "a6x")
                this->a6x = stod(par_val);
        }
        cout << " load completed!" << endl;
    }
    else{
        cout << " Error while opening file!" << '\n';
    }
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

void Robot::setdimensionsKUKA(void){
    this->a1z = 0.0;
    this->a2x = 175.0;
    this->a2z = 575.0;
    this->a3z = 890.0;
    this->a4x = 1035.0;
    this->a4z = 50.0;
    this->a5x = 0.0;
    this->a6x = 185.0;
    this->brand = KUKA;
}

void Robot::printAffine3d(Eigen::Affine3d p){
    Eigen::Vector3d t = p.translation();
    Eigen::Quaterniond q(p.rotation());
    Eigen::Vector3d r;

    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    std::cout << "X: " << t.x();
    std::cout << "; Y: " << t.y();
    std::cout << "; Z: " << t.z() << std::endl;
    switch (this->brand)
    {
    case IR:
        r = p.rotation().eulerAngles(0,1,2) * 180.0 / M_PI;
        std::cout << "RX: " << r.x();
        std::cout << "; RY: " << r.y();
        std::cout << "; RZ: " << r.z() << std::endl;
        break;
    case ABB:
        std::cout << std::setprecision(6);
        std::cout << "q1: " << q.w();
        std::cout << "; q2: " << q.x();
        std::cout << "; q3: " << q.y();
        std::cout << "; q4: " << q.z() << std::endl;
        break;
    case KUKA:
        r = p.rotation().eulerAngles(2,1,0) * 180.0 / M_PI;
        std::cout << "A: " << r.z();
        std::cout << "; B: " << r.y();
        std::cout << "; C: " << r.x() << std::endl;
        break;
    }
}

Eigen::Affine3d Robot::FK(Joint j_, Eigen::Affine3d UT, Eigen::Affine3d UF){
    Joint j = j_;
    switch(this->brand)
    {
        case IR:
            std::cout << "FK IR\n";
            break;
        case ABB:
            std::cout << "FK ABB\n";
            break;
        case KUKA:
            std::cout << "FK KUKA\n";
            j.setj1(-j.getj1());
            j.setj2(j.getj2() + M_PI_2);
            j.setj3(j.getj3() - M_PI_2);
            j.setj4(-j.getj4());
            j.setj6(-j.getj6());
            break;
    }
    
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
            break;
        case ABB:
            // Rotation 90° around Y
            l16.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
            break;
        case KUKA:
            // Rotation 90° around Y
            l16.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
            break;
    }

    return UF.inverse() * l16 * UT;
}

Joint Robot::IK(Eigen::Affine3d p, Eigen::Affine3d UT, Eigen::Affine3d UF, Joint jAct, FrontBack FB, UpDown UD, PositiveNegative PN){
    double J1, J2, J3, J4, J5, J6;
    double Rwrist11, Rwrist21, Rwrist31, Rwrist12, Rwrist13, Rwrist32, Rwrist33;
    double WPxy, l, h;
    double rho, b4x;
    double alpha, beta, gamma, delta;
    double cos_beta, sin_beta;
    double cos_gamma, sin_gamma;
    Eigen::Affine3d MP, pJ1, pJ23;
    Eigen::Vector3d x_hat, WP;
    Eigen::Matrix3d Rarm, Rwrist;

    //Pose of the mounting point (center of the axe 6)
    MP = UF * p * UT.inverse();

    switch(this->brand)
    {
        case IR:
            std::cout << "IK IR\n";
            break;
        case ABB:
            std::cout << "IK ABB\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
            break;
        case KUKA:
            std::cout << "IK KUKA\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
            break;
    }

    x_hat = MP.rotation() * Eigen::Vector3d::UnitX();
    WP = MP.translation() - (this->a6x * x_hat);

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
        if(FB == Back){
            // BACK solution is selected
            if(J1 > 0)
                J1 -= M_PI;
            else
                J1 += M_PI;
        }
    }

    // Find J2 and J3
    WPxy = sqrt( pow(WP(0),2) + pow(WP(1),2) );

    switch (FB)
    {
    case Front:
        // FRONT solution
        l = WPxy - this->a2x;
        break;
    case Back:
        // BACK solution
        l = WPxy + this->a2x;
        break;
    }
    h = WP(2) - this->a1z - this->a2z;

    rho = sqrt( pow(h,2) + pow(l,2) );
    b4x = sqrt( pow(this->a4z,2) + pow(this->a4x+this->a5x,2) );
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

    switch (UD)
    {
    case Up:
        // UP solution
        J2 = M_PI_2 - alpha - beta;
        break;
    case Down:
        // DOWN solution
        J2 = M_PI_2 - alpha + beta;
        break;
    }

    cos_gamma = (pow(this->a3z,2) + pow(b4x,2) - pow(rho,2)) / (2*this->a3z*b4x);
    sin_gamma = sqrt(1 - pow(cos_gamma,2));
    gamma = atan2(sin_gamma, cos_gamma);
    delta = atan2(this->a4x+this->a5x, this->a4z);

    J3 = M_PI - gamma - delta;

    // Calculate Rarm from the values of J1, J2, J3
    pJ1 = Eigen::AngleAxisd(J1, Eigen::Vector3d::UnitZ());
    pJ23 = Eigen::AngleAxisd(J2+J3, Eigen::Vector3d::UnitY());
    Rarm = pJ1.rotation() * pJ23.rotation();
    // R = Rarm * Rwrist -> Rwrist = Rarm^T * R
    Rwrist = Rarm.transpose() * MP.rotation();

    //Find J4, J5, J6 from Rwrist
    Rwrist11 = Rwrist(0,0);
    Rwrist21 = Rwrist(1,0);
    Rwrist31 = Rwrist(2,0);
    Rwrist12 = Rwrist(0,1);
    Rwrist13 = Rwrist(0,2);
    Rwrist32 = Rwrist(2,1);
    Rwrist33 = Rwrist(2,2);

    if (Rwrist11 < 0.9999999) {
        if (Rwrist11 > -0.9999999) {
            switch (PN)
            {
            case Positive:
                J5 = atan2( sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                J4 = atan2(Rwrist21,-Rwrist31);
                J6 = atan2(Rwrist12,Rwrist13);
                break;
            case Negative:
                J5 = atan2( -sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                J4 = atan2(-Rwrist21,Rwrist31);
                J6 = atan2(-Rwrist12,-Rwrist13);
                break;
            }
        }
        else // Rwrist11 = −1 
        {
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
        // Wrist singularity. J5 = 0
        // Not a unique solution: J4 + J6 = atan2(Rwrist32,Rwrist33)
        J5 = 0;
        J4 = jAct.getj4();
        J6 = atan2(Rwrist32,Rwrist33) - J4;
    }

    switch(this->brand)
    {
        case IR:
            break;
        case ABB:
            break;
        case KUKA:
            J1 = -J1;
            J2 -= M_PI_2;
            J3 += M_PI_2;
            J4 = -J4;
            J6 = -J6;
            break;
    }

    Joint j(J1, J2, J3, J4, J5, J6);
    return j;
}