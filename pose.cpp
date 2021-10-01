#include <iostream>
#include <iomanip>
#include "pose.h"

Pose::Pose(void){
      this->pos = Vector3d::Zero();
      this->rot = (Vector4d() << 0, 0, 0, 1).finished();
}

Pose::Pose(Vector3d pos, Vector4d rot){
      this->pos = pos;
      this->rot = rot;
}

Pose::Pose(Vector3d pos, Matrix3d R){
      this->pos = pos;
      this->rot = this->RtoQ(R);
}

Pose::Pose(Matrix4d T){
      this->pos = T.topRightCorner(3,1);
      this->rot = this->RtoQ(T.topLeftCorner(3,3));
}

void Pose::setpos(double x, double y, double z){
      this->pos << x, y, z;
}

void Pose::setpos(Vector3d pos){
      this->pos = pos;
}

void Pose::setrot(double x, double y, double z, double w){
      this->rot << x, y, z, w;
}

void Pose::setrotFromABC_rad_(double A, double B, double C){
      this->rot = this->RtoQ(this->ABC_rad_toR((Array3d() << A, B, C).finished()));
}

void Pose::setrotFromR(Matrix3d R){
      this->rot = this->RtoQ(R);
}

Vector3d Pose::getpos(){
      return this->pos;
}

Vector4d Pose::getrot(){
      return this->rot;
}

Vector3d Pose::getrotABC_rad(){
      return this->RtoABC_rad_(this->QtoR(this->rot));
}

// Rotation matrix around X. Angle in radiants
Matrix3d Pose::RaroundX(double A){
      Matrix3d R;
      // Rotation around X
      R  << 1, 0     , 0      ,
            0, cos(A), -sin(A),
            0, sin(A),  cos(A);
      return R;
}

// Rotation matrix around Y. Angle in radiants
Matrix3d Pose::RaroundY(double B){
      Matrix3d R;
      // Rotation around Y
      R  << cos(B) , 0, sin(B),
            0      , 1, 0     ,
            -sin(B), 0, cos(B);
      return R;
}

// Rotation matrix around Z. Angle in radiants
Matrix3d Pose::RaroundZ(double C){
      Matrix3d R;
      // Rotation around Z
      R  << cos(C), -sin(C), 0,
            sin(C), cos(C) , 0,
            0     , 0      , 1;
      return R;
}

Vector4d Pose::RtoQ(Matrix3d R){
      // Transform rotation matrix to quaternion
      double x, y, z, w;
      double R11, R21, R31, R12, R22, R32, R13, R23, R33;
      // Read components of rotation matrix
      R11 = R(0,0);
      R21 = R(1,0);
      R31 = R(2,0);
      R12 = R(0,1);
      R22 = R(1,1);
      R32 = R(2,1);
      R13 = R(0,2);
      R23 = R(1,2);
      R33 = R(2,2);
      // w = 1/2 * sqrt(1 + R11 + R22 + R33)
      w = 0.5 * sqrt(1+R11+R22+R33);
      // x = 1/4w * (R32 - R23)
      x = (R32-R23) / (4.0*w);
      // y = 1/4w * (R13 - R31)
      y = (R13-R31) / (4.0*w);
      // z = 1/4w * (R21 - R12)
      z = (R21-R12) / (4.0*w);
      // Copy value in a 4 dim vector
      Vector4d Q;
      Q << x, y, z, w;
      // Return vector
      return Q;
}

Matrix3d Pose::QtoR(Vector4d Q){
      // Transform quaternion to rotation matrix
      double x, y, z, w;
      double R11, R21, R31, R12, R22, R32, R13, R23, R33;
      // Read components of quaternion
      x = Q(0);
      y = Q(1);
      z = Q(2);
      w = Q(3);
      // R11 = 1 - 2y^2 - 2z^2
      R11 = 1 - 2*pow(y,2) - 2*pow(z,2);
      // R21 = 2xy + 2zw
      R21 = 2*x*y + 2*z*w;
      // R31 = 2xz - 2yw
      R31 = 2*x*z - 2*y*w;
      // R12 = 2xy - 2zw
      R12 = 2*x*y - 2*z*w;
      // R22 = 1 - 2x^2 - 2z^2
      R22 = 1 - 2*pow(x,2) - 2*pow(z,2);
      // R32 = 2yz + 2xw
      R32 = 2*y*z + 2*x*w;
      // R13 = 2xz + 2yw
      R13 = 2*x*z + 2*y*w;
      // R23 = 2yz - 2xw
      R23 = 2*y*z - 2*x*w;
      // R33 = 1 - 2x^2 - 2y^2
      R33 = 1 - 2*pow(x,2) - 2*pow(y,2);
      // Define rotation matrix
      Matrix3d R;
      R <<  R11, R12, R13,
            R21, R22, R23,
            R31, R32, R33;
      // Return rotation matrix
      return R;
}

// A, B, C (in radiants) to rotation matrix R
Matrix3d Pose::ABC_rad_toR(Vector3d ABC){
      // Get the rotation matrix from euler angles A, B, C
      double A, B, C;
      Matrix3d RX, RY, RZ, R;
      // Read components of rotation matrix
      A = ABC(0);
      B = ABC(1);
      C = ABC(2);
      // Rotation around X
      RX = RaroundX(A);
      // Rotation around Y
      RY = RaroundY(B);
      // Rotation around Z
      RZ = RaroundZ(C);
      // Create the rotation matrix
      R = RZ*RY*RX;
      // Return the rotation matrix
      return R;
}

// Rotation matrix R to Euler angles A, B, C (in radiants)
Vector3d Pose::RtoABC_rad_(Matrix3d R){
      // Get euler angles A, B, C from rotation matrix
      double A, B, C;
      double R11, R21, R31, R12, R22, R32, R13, R23, R33;
      // Read components of rotation matrix
      R11 = R(0,0);
      R21 = R(1,0);
      R31 = R(2,0);
      R12 = R(0,1);
      R22 = R(1,1);
      R32 = R(2,1);
      R13 = R(0,2);
      R23 = R(1,2);
      R33 = R(2,2);

      if (R31 < 1.0){
            if (R31 > -1.0)
            {
                  B = asin(-R31);
                  C = atan2(R21, R11);
                  A = atan2(R32, R33);
            }
            else // R31 = −1
            {
                  // Not a unique solution: A − C = atan2(−R23,R22)
                  B = M_PI_2;
                  C = atan2(-R23, R22);
                  A = 0.0;
            }
      }
      else // R31 = +1
      {
            // Not a unique solution: A + C = atan2(−R23,R22)
            B = -M_PI_2;
            C = atan2(-R23, R22);
            A = 0.0;
      }
      // Copy value in a 3 dim vector
      Vector3d ABC;
      ABC << A, B, C;
      // Return the vector
      return ABC;
}

Matrix3d Pose::getR(){
      return this->QtoR(this->rot);
}

Matrix4d Pose::getT(){
      Matrix4d T;
      T <<  this->getR(), this->pos,
            0, 0, 0, 1;
      return T;
}

void Pose::print(void){
      std::cout << std::fixed;
      std::cout << std::setprecision(2);
      std::cout << "pos x: "   << this->pos(0);
      std::cout << "; y: " << this->pos(1);
      std::cout << "; z: " << this->pos(2);
      std::cout << std::endl;
      std::cout << std::setprecision(6);
      std::cout << "rot x: "   << this->rot(0);
      std::cout << "; y: " << this->rot(1);
      std::cout << "; z: " << this->rot(2);
      std::cout << "; w: " << this->rot(3);
      std::cout << std::endl;
}

void Pose::printABC_deg_(void){
      Array3d ABC;
      ABC = getrotABC_rad() * 180.0 / M_PI;
      std::cout << std::fixed;
      std::cout << std::setprecision(2);
      std::cout << "pos x: "   << this->pos(0);
      std::cout << "; y: " << this->pos(1);
      std::cout << "; z: " << this->pos(2);
      std::cout << std::endl;
      std::cout << std::setprecision(2);
      std::cout << "rot A: "   << ABC(0);
      std::cout << "; B: " << ABC(1);
      std::cout << "; C: " << ABC(2);
      std::cout << std::endl;
}
