#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "robot.h"
#include "pose.h"
#include "joint.h"

using namespace std;
using namespace Eigen;

// BT (BackTransformation) is used to convert a Pose back using the invert of the
// transformation poseT in input.
// ATTENZIONE: NON FUNZIONA CORRETTAMENTE!!!!
// CI DEVE ESSERE UN BUG DA QUALCHE PARTE!!!!
Pose BT(Pose p, Pose poseT){
      Vector3d MP_XYZ;
      Matrix3d MP_R;

      Matrix3d R_MP_tool0 = poseT.getR();
      Matrix4d T_tool0_MP;
      Vector3d MP_to_tool0_pos = poseT.getpos();
      T_tool0_MP << R_MP_tool0.transpose(), -(R_MP_tool0.transpose()) * MP_to_tool0_pos,
            0, 0, 0, 1;
      //std::cout << "T_tool0_MP: " << std::endl << T_tool0_MP << std::endl;

      Matrix4d T_TCP_XYZ;
      T_TCP_XYZ << MatrixXd::Identity(3,3), p.getpos(),
            0, 0, 0, 1;
      //std::cout << "T_TCP_XYZ: " << std::endl << T_TCP_XYZ << std::endl;

      // Mounting point MP (Center of the flange of axis 6)      
      MP_XYZ << (T_TCP_XYZ * T_tool0_MP).topRightCorner(3,1);
      MP_R =  p.getR() * R_MP_tool0.transpose();
      Pose MP(MP_XYZ, MP_R);

      return MP;
}

int main(){
      Joint j, jpos0, jpos1, jpos2, jpos3, jpos4, jpos5, jIK;
      Pose p;
      Pose UT0, UT1;
      Pose UF0, UF1;
      Pose MP_to_tool0;
      Robot Rob;

      // Robot Industrial robot
      Rob.setdimensionsIR();

      // Joint values
      jpos0.setall_deg_(0, 0, 0, 0, 0, 0);
      jpos1.setall_deg_(90, 0, 0, 0, 0, 0);
      jpos2.setall_deg_(45, 45, 45, 45, 45, 45);
      jpos3.setall_deg_(-45, -45, -45, -45, -45, -45);
      jpos4.setall_deg_(130, -60, 30, 60, -90, 60);
      jpos5.setall_deg_(-46, 46, 46, 46, 46, 46);

      // Set UT values
      //UT0: X=0; Y=0; Z=0; A=RX=0; B=RY=0; C=RZ=0;
      UT0.setpos(0.0, 0.0, 0.0);
      UT0.setrotFromABC_rad_(0.0, 0.0, 0.0);
      //UT1: X=200; Y=100; Z=300; A=RX=0; B=RY=60; C=RZ=0;
      UT1.setpos(200.0, 100.0, 300.0);
      UT1.setrotFromABC_rad_(0.0, 60.0*(M_PI/180.0), 0.0);

      // Set UF values
      //UF0: X=0; Y=0; Z=0; A=RX=0; B=RY=0; C=RZ=0;
      UF0.setpos(0.0, 0.0, 0.0);
      UF0.setrotFromABC_rad_(0.0, 0.0, 0.0);
      //UF1: X=1000; Y=-500; Z=750; A=RX=0; B=RY=0; C=RZ=45;
      UF1.setpos(1000.0, -500.0, 750.0);
      UF1.setrotFromABC_rad_(0.0, 0.0, 45.0*(M_PI/180.0));

      // UT=tool0; UF=wobj0; X=2006.67, Y=20.56, Z=1140.25, A=125.41, B=-18.77, C=93.93;

      j = jpos5;

      // Direct kinematic
      std::cout << "FK - Direct kinematic" << std::endl;
      p = Rob.FK(j, MP_to_tool0);
      j.print_deg_();
      p.printABC_deg_();
      
      // Inverse kinematic
      // p = TCP position and orientation (in mm and degree)
      // j = Actual values of joints
      // Front/back configuration : front = false
      // Up/down configuration : up = false
      bool FrontBack = false;
      bool UpDown = false;
      std::cout << "IK - Inverse kinematic" << std::endl;
      jIK = Rob.IK(p, j, FrontBack, UpDown, MP_to_tool0);
      p.printABC_deg_();
      jIK.print_deg_();
}