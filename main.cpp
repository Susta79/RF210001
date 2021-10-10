#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "robot.h"
#include "joint.h"

void printAffine3d(Eigen::Affine3d p){
      Eigen::Vector3d t = p.translation();
      Eigen::Vector3d r = p.rotation().eulerAngles(2,1,0) * 180.0 / M_PI;
      Eigen::Quaterniond q(p.rotation());
      
      std::cout << std::fixed;
      std::cout << std::setprecision(2);
      std::cout << "X: " << t.x();
      std::cout << "; Y: " << t.y();
      std::cout << "; Z: " << t.z() << std::endl;
      std::cout << "RZ: " << r.z();
      std::cout << "; RY: " << r.y();
      std::cout << "; RX: " << r.x() << std::endl;

      std::cout << std::setprecision(6);
      std::cout << "Quaternion: " << q << std::endl;
}

int main(){
      Joint j, jIK2;
      Joint jpos00, jpos01, jpos02, jpos03, jpos04, jpos05, jpos06;
      Joint jpos07, jpos08, jpos09, jpos10, jpos11, jpos12;
      Robot Rob;

      // Set UT values
      //UT0: X=0; Y=0; Z=0; A=RX=0; B=RY=0; C=RZ=0;
      Eigen::Affine3d UT0 = Eigen::Affine3d::Identity();
      //UT1: X=200; Y=100; Z=300; A=RX=0; B=RY=60; C=RZ=0;
      Eigen::Affine3d UT1t(Eigen::Translation3d(Eigen::Vector3d(200, 100, 300)));
      Eigen::Affine3d UT1r(Eigen::AngleAxisd(60.0*M_PI/180.0, Eigen::Vector3d::UnitY()));
      Eigen::Affine3d UT1 = UT1t * UT1r;

      // Set UF values
      //UF0: X=0; Y=0; Z=0; A=RX=0; B=RY=0; C=RZ=0;
      Eigen::Affine3d UF0 = Eigen::Affine3d::Identity();
      //UF1: X=1000; Y=-500; Z=750; A=RX=0; B=RY=0; C=RZ=45;
      Eigen::Affine3d UF1t(Eigen::Translation3d(Eigen::Vector3d(1000, -500, 750)));
      Eigen::Affine3d UF1r(Eigen::AngleAxisd(45.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));
      Eigen::Affine3d UF1 = UF1t * UF1r;

      // Robot Industrial robot
      Rob.setdimensionsABB();

      // Joint values
      jpos00.setall_deg_(  0,  0,  0,  0,  0,  0);
      jpos01.setall_deg_(-45,  0,  0,  0,  0,  0);
      jpos02.setall_deg_( 45,  0,  0,  0,  0,  0);
      jpos03.setall_deg_(  0,-45,  0,  0,  0,  0);
      jpos04.setall_deg_(  0, 45,  0,  0,  0,  0);
      jpos05.setall_deg_(  0,  0,-45,  0,  0,  0);
      jpos06.setall_deg_(  0,  0, 45,  0,  0,  0);
      jpos07.setall_deg_(  0,  0,  0,-45,  0,  0);
      jpos08.setall_deg_(  0,  0,  0, 45,  0,  0);
      jpos09.setall_deg_(  0,  0,  0,  0,-45,  0);
      jpos10.setall_deg_(  0,  0,  0,  0, 45,  0);
      jpos11.setall_deg_(  0,  0,  0,  0,  0,-45);
      jpos12.setall_deg_(  0,  0,  0,  0,  0, 45);

      j = jpos01;

      // Direct kinematic
      std::cout << "FK2 - Direct kinematic" << std::endl;
      Eigen::Affine3d fk2= Rob.FK2(j, UF0);
      printAffine3d(fk2);
      
      // Inverse kinematic
      // p = TCP position and orientation (in mm and degree)
      // j = Actual values of joints
      // Front/back configuration : front = false
      // Up/down configuration : up = false
      bool FrontBack = false;
      bool UpDown = false;
      std::cout << "IK2 - Inverse kinematic" << std::endl;
      jIK2 = Rob.IK2(fk2, j, FrontBack, UpDown);
      jIK2.print_deg_();
}
