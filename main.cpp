#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <iomanip>
#include <Eigen/Dense>
#include "robot.h"
#include "joint.h"

namespace fs = std::filesystem;

int main(){
      //for (const auto & entry : fs::directory_iterator(p1.c_str()))
      //      std::cout << entry.path() << std::endl;
      fs::path pathCfgFolder = fs::current_path().concat("/../cfg/");
      fs::path pathCfgFile = pathCfgFolder.concat("ABB_4600_20_250.cfg");
      std::ifstream infile(pathCfgFile.c_str());
      std::string line;
      if(infile.is_open()){
            while (std::getline(infile, line)){
                  std::cout << line << '\n';
                  size_t f = line.find_first_of(" ");
                  std::cout << "Fisrt part: " << line.substr(0,f) << '\n';
                  std::cout << "Second part: " << line.substr(f+1,line.size()-f) << '\n';
            }
      }
      else{
            std::cout << "File not open!" << '\n';
      }
      
      Joint j, jIK;
      Joint jpos00, jpos01, jpos02, jpos03, jpos04, jpos05, jpos06;
      Joint jpos07, jpos08, jpos09, jpos10, jpos11, jpos12;
      Joint jpos20, jpos21;
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

      // Robot
      Rob.setdimensionsKUKA();

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

      jpos20.setall_deg_(  0,-90, 90,  0,  0,  0);
      jpos21.setall_deg_(  -61.706, -79.587, 100.372,  -90.605,  88.405, 60.794);

      j = jpos21;

      // Direct kinematic
      std::cout << "FK - Direct kinematic" << std::endl;
      j.print_deg_();
      Eigen::Affine3d fk= Rob.FK(j, UT0, UF0);
      Rob.printAffine3d(fk);
      
      // Inverse kinematic
      // p = TCP position and orientation (in mm and degree)
      // j = Actual values of joints
      std::cout << "IK - Inverse kinematic" << std::endl;
      jIK = Rob.IK(fk, UT0, UF0, j, Front, Up, Positive);
      jIK.print_deg_();
}
