#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>

using namespace Eigen;

class Pose {
private:
      // x y z
      Vector3d pos;
      // x y z w
      Vector4d rot;
      Matrix3d RaroundX(double);
      Matrix3d RaroundY(double);
      Matrix3d RaroundZ(double);
      Vector4d RtoQ(Matrix3d);
      Matrix3d QtoR(Vector4d);
      Matrix3d ABC_rad_toR(Vector3d);
      Vector3d RtoABC_rad_(Matrix3d);
public:
      Pose();
      Pose(Vector3d, Vector4d);
      Pose(Vector3d, Matrix3d);
      Pose(Matrix4d);
      void setpos(Vector3d);
      void setpos(double, double, double);
      void setrot(double, double, double, double);
      void setrotFromABC_rad_(double, double, double);
      void setrotFromR(Matrix3d);
      Vector3d getpos();
      Vector4d getrot();
      Vector3d getrotABC_rad();
      Matrix3d getR();
      Matrix4d getT();
      void print();
      void printABC_deg_();
};

#endif