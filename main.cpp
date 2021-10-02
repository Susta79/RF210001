#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "pose.h"
#include "joint.h"

using namespace std;
using namespace Eigen;

// ABB 4600-20/2.50
#define a1z 0.0
#define a2x 175.0
#define a2z 495.0
#define a3z 1095.0
#define a4x 1230.5
#define a4z 175.0
#define a5x 0.0
#define a6x 85.0

enum Brand { IR, ABB, KUKA };

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

// j: joint values in radiants
// MP_to_tool0: transformation from MP (mounting point = center of axe 6) to tool0
Pose FK(Joint j, Brand brand, Pose MP_to_tool0){
      Pose VTR;
      Pose link1, link2, link3, link4, link5, link6;
      Matrix4d T16, T;
      Matrix3d Rarm, Rwrist, R16, R;

      // Link1: Base to Joint 1
      // Translation
      link1.setpos(0.0, 0.0, a1z);
      // Rotation around Z
      link1.setrotFromABC_rad_(0.0, 0.0, j.getj1());

      // Link2: Joint 1 to Joint 2
      // Translation
      link2.setpos(a2x, 0.0, a2z);
      // Rotation around Y
      link2.setrotFromABC_rad_(0.0, j.getj2(), 0.0);

      // Link3: Joint 2 to Joint 3
      // Translation
      link3.setpos(0.0, 0.0, a3z);
      // Rotation around Y
      link3.setrotFromABC_rad_(0.0, j.getj3(), 0.0);

      // Link4: Joint 3 to Joint 4
      // Translation
      link4.setpos(a4x, 0.0, a4z);
      // Rotation around X
      link4.setrotFromABC_rad_(j.getj4(), 0.0, 0.0);

      // Link5: Joint 4 to Joint 5
      // Translation
      link5.setpos(a5x, 0, 0);
      // Rotation around Y
      link5.setrotFromABC_rad_(0.0, j.getj5(), 0.0);

      // Link6: Joint 5 to Joint 6
      // Translation
      link6.setpos(a6x, 0, 0);
      // Rotation around X
      link6.setrotFromABC_rad_(j.getj6(), 0.0, 0.0);

      //T16 = T1*T2*T3*T4*T5*T6;
      T16 = link1.getT() * link2.getT() * link3.getT() * link4.getT() * link5.getT() * link6.getT();

      Matrix4d Ttool0;
      //Ttool0 = XYZABCtoT(tool0);
      T = T16 * MP_to_tool0.getT();
      //T = T16;

      Vector4d O;
      O << 0,0,0,1;
      Vector3d XYZ = (T*O).head(3);

      //Rarm = R1*R2*R3;
      Rarm = link1.getR() * link2.getR() * link3.getR();
      //Rwrist = R4*R5*R6;
      Rwrist = link4.getR() * link5.getR() * link6.getR();
      R16 = Rarm * Rwrist;

      Pose Rtmp;
      switch(brand)
      {
            case IR:
                  std::cout << "FK IR\n";
                  R = R16;
                  break;
            case ABB:
                  std::cout << "FK ABB\n";
                  // Rotation 90° around Y
                  Rtmp.setrotFromABC_rad_(0.0, M_PI/2.0, 0.0);
                  R = R16 * Rtmp.getR();
                  break;
            case KUKA:
                  std::cout << "FK KUKA\n";
                  R = R16;
                  break;
      }

      VTR.setpos(XYZ);
      VTR.setrotFromR(R);

      return VTR;
}

Joint IK(Pose p, Joint jAct, bool bFrontBack, bool bUpDown, Brand brand, Pose MP_to_tool0){
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

      switch(brand)
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
                  Rtmp.setrotFromABC_rad_(0.0, M_PI/2.0, 0.0);
                  MP.setrotFromR(Rtmp.getR().transpose() * p.getR());
                  break;
            case KUKA:
                  std::cout << "IK KUKA\n";
                  MP = p;
                  break;
      }

      x_hat = MP.getR() * ((Vector3d() << 1, 0, 0).finished());
      WP = MP.getpos() - (a6x * x_hat);

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
            l = WPxy + a2x;
      }
      else{
            // FRONT solution
            l = WPxy - a2x;
      }
      //std::cout << "l:\n" << l << std::endl;
      h = WP(2) - a1z - a2z;
      //std::cout << "h:\n" << h << std::endl;

      rho = sqrt( pow(h,2) + pow(l,2) );
      //std::cout << "rho:\n" << rho << std::endl;
      b4x = sqrt( pow(a4z,2) + pow(a4x+a5x,2) );
      //std::cout << "b4x:\n" << b4x << std::endl;
      if(rho >= (a3z+b4x)){
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

      delta = atan2(a4x+a5x, a4z);

      J3 = M_PI - gamma - delta;

      //std::cout << "J1:\n" << J1 << std::endl;
      //std::cout << "J2:\n" << J2 << std::endl;
      //std::cout << "J3:\n" << J3 << std::endl;

      Pose pJ1, pJ23;
      pJ1.setrotFromABC_rad_(0.0, 0.0, J1);
      pJ23.setrotFromABC_rad_(0.0, (J2+J3), 0.0);
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

      if (Rwrist11 < 1.0) {
            if (Rwrist11 > -1.0) {
                  //J5 = acos(Rwrist11);
                  J5 = atan2( sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                  J4 = atan2(Rwrist21,-Rwrist31);
                  J6 = atan2(Rwrist12,Rwrist13);
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

      Joint j(J1, J2, J3, J4, J5, J6);
      return j;
}

int main(){
      Joint j, jpos0, jpos1, jpos2, jpos3, jIK;
      Pose p;
      Pose UT0, UT1;
      Pose UF0, UF1;
      Pose MP_to_tool0;

      // Joint values
      jpos0.setall_deg_(0, 0, 0, 0, 0, 0);
      jpos1.setall_deg_(90, 0, 0, 0, 0, 0);
      jpos2.setall_deg_(45, 45, 45, 45, 45, 45);
      jpos3.setall_deg_(-45, -45, -45, -45, -45, -45);

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

      // Direct kinematic
      // Robot IR
      //Joint << 0, 0, 0, 0, 0, 0;
      //Joint << 90, 0, 0, 0, 0, 0;
      //Joint << 130, -60, 30, 60, -90, 60;
      //Joint << -46, 46, 46, 46, 46, 46;

      j = jpos0;
      std::cout << "FK - Direct kinematic" << std::endl;
      p = FK(j, ABB, MP_to_tool0);
      j.print_deg_();
      p.printABC_deg_();
      
      // Inverse kinematic
      // Robot IR
      //XYZABC << 1755, 0, 2660, 0, 0, 0;
      //XYZABC << 1500, 1000, 2000, 0, 0, 0;
      //XYZABC << -500, 1000, 2000, 50, 50, 50;
      //XYZABC << 600, -1000, 3300, 250, 0, -90;
      // ABB IRB 4600-20/2.50
      //XYZABC << 1490.5, 0, 1765, 0, 90, 0; // Joint = 0 0 0 0 0 0
      //XYZABC << 1405.5, 0, 1680, -180, 0, -180; // Joints = 0 0 0 0 90 0
      //XYZABC << 2006.67, 20.56, 1140.25, 125.41, -18.77, 93.93; // Joints = 0 30 -10 45 20 60
      //p.setpos(1490.5, 0, 1765);
      //p.setrotFromABC_rad_(0.0, 90.0  * (M_PI/180.0), 0.0); // Joint = 0 0 0 0 0 0

      // XYZABC = TCP position and orientation (in mm and degree)
      // JointAct = Actual values of joints
      // Front/back configuration : front = false
      // Up/down configuration : up = false
      // brand : IR (Industrial Robotics), ABB or KUKA
      // tool0 is the transformation to tool 0
      bool FrontBack = true;
      bool UpDown = true;
      std::cout << "IK - Inverse kinematic" << std::endl;
      jIK = IK(p, j, FrontBack, UpDown, ABB, MP_to_tool0);
      p.printABC_deg_();
      jIK.print_deg_();
}