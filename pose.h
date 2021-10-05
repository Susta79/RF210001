#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>

using namespace Eigen;

class Pose {
private:
      // x y z
      Vector3d pos;
      // x y z w
      Quaterniond rot;
      Matrix3d RaroundX(double);
      Matrix3d RaroundY(double);
      Matrix3d RaroundZ(double);      
      Vector4d RtoQ(Matrix3d);
      Matrix3d QtoR(Vector4d);
      //Matrix3d ABC_rad_toR(Vector3d);
      Vector3d RtoEuler(Matrix3d);
public:
      Pose();
      Pose(Vector3d, Quaterniond);
      Pose(Vector3d, Matrix3d);
      Pose(Matrix4d);
      void setpos(Vector3d);
      void setpos(double x, double y, double z);
      void setrot(double x, double y, double z, double w);
      //void setrotFromABC_rad_(double, double, double);
      void setrot(double RX, double RY, double RZ);
      void setrot(Matrix3d R);
      Vector3d getpos();
      Quaterniond getrot();
      //Vector3d getrotABC_rad();
      Vector3d getrotEuler();
      Matrix3d getR();
      Matrix4d getT();
      void print();
      void printABC_deg_();
};

#endif