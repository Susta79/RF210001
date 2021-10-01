#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "joint.h"

using namespace Eigen;
using namespace std;

Joint::Joint(void){
      this->j1 = 0;
      this->j2 = 0;
      this->j3 = 0;
      this->j4 = 0;
      this->j5 = 0;
      this->j6 = 0;
}

Joint::Joint(double j1, double j2, double j3, double j4, double j5, double j6){
      this->j1 = j1;
      this->j2 = j2;
      this->j3 = j3;
      this->j4 = j4;
      this->j5 = j5;
      this->j6 = j6;
}

double Joint::getj1(){ return this->j1; }
double Joint::getj2(){ return this->j2; }
double Joint::getj3(){ return this->j3; }
double Joint::getj4(){ return this->j4; }
double Joint::getj5(){ return this->j5; }
double Joint::getj6(){ return this->j6; }
void Joint::setj1(double j){ this->j1 = j; }
void Joint::setj2(double j){ this->j2 = j; }
void Joint::setj3(double j){ this->j3 = j; }
void Joint::setj4(double j){ this->j4 = j; }
void Joint::setj5(double j){ this->j5 = j; }
void Joint::setj6(double j){ this->j6 = j; }

void Joint::setall_deg_(double j1, double j2, double j3, double j4, double j5, double j6){
      this->j1 = j1 * (M_PI / 180.0);
      this->j2 = j2 * (M_PI / 180.0);
      this->j3 = j3 * (M_PI / 180.0);
      this->j4 = j4 * (M_PI / 180.0);
      this->j5 = j5 * (M_PI / 180.0);
      this->j6 = j6 * (M_PI / 180.0);
}

void Joint::print_deg_(void){
      std::cout << std::fixed;
      std::cout << std::setprecision(2);
      std::cout << "j1: "   << this->j1 * (180.0 / M_PI);
      std::cout << "; j2: " << this->j2 * (180.0 / M_PI);
      std::cout << "; j3: " << this->j3 * (180.0 / M_PI);
      std::cout << "; j4: " << this->j4 * (180.0 / M_PI);
      std::cout << "; j5: " << this->j5 * (180.0 / M_PI);
      std::cout << "; j6: " << this->j6 * (180.0 / M_PI);
      std::cout << endl;
}