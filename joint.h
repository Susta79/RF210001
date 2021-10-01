#ifndef JOINT_H
#define JOINT_H

#include <Eigen/Dense>

using namespace Eigen;

class Joint {
private:
      // Internal values in radiants
      double j1, j2, j3, j4, j5, j6;
public:
      Joint();
      Joint(double, double, double, double, double, double);
      double getj1();
      double getj2();
      double getj3();
      double getj4();
      double getj5();
      double getj6();
      void setj1(double);
      void setj2(double);
      void setj3(double);
      void setj4(double);
      void setj5(double);
      void setj6(double);
      void setall_deg_(double, double, double, double, double, double);
      void print_deg_();
};

#endif