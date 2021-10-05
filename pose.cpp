#include <iostream>
#include <iomanip>
#include "pose.h"

Pose::Pose(void){
      this->pos = Vector3d::Zero();
      // x, y, z, w
      this->rot.setIdentity();
}

Pose::Pose(Vector3d pos, Quaterniond rot){
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
      this->rot = Quaterniond(w, x, y, z);
}

void Pose::setrot(double RX, double RY, double RZ){
      double x, y, z, w;
      // Abbreviations for the various angular functions
      double cy = cos(RZ * 0.5);
      double sy = sin(RZ * 0.5);
      double cp = cos(RY * 0.5);
      double sp = sin(RY * 0.5);
      double cr = cos(RX * 0.5);
      double sr = sin(RX * 0.5);

      x = sr * cp * cy - cr * sp * sy;
      y = cr * sp * cy + sr * cp * sy;
      z = cr * cp * sy - sr * sp * cy;
      w = cr * cp * cy + sr * sp * sy;

      this->rot = Quaterniond(w, x, y, z);
}

void Pose::setrot(Matrix3d R){
      this->rot = RtoQ(R);
}

Vector3d Pose::getpos(){
      return this->pos;
}

Quaterniond Pose::getrot(){
      return this->rot;
}

//Vector3d Pose::getrotABC_rad(){
//      return this->RtoEuler(this->QtoR(this->rot));
//}

/*
Vector3d Pose::getrotEuler(){
      Vector3d angles;
      double t0, t1, t2, t3, t4;
      double x, y, z, w;
      double RX, RY, RZ;
      x = this->rot(0);
      y = this->rot(1);
      z = this->rot(2);
      w = this->rot(3);

      t0 = +2.0 * (w * x + y * z);
      t1 = +1.0 - 2.0 * (x * x + y * y);
      RX = atan2(t0, t1);
      t2 = +2.0 * (w * y - z * x);
      if (t2 > +1.0)
            t2 = +1.0;
      if (t2 < -1.0)
            t2 = -1.0;
      RY = asin(t2);
      t3 = +2.0 * (w * z + x * y);
      t4 = +1.0 - 2.0 * (y * y + z * z);
      RZ = atan2(t3, t4);
      angles << RX, RY, RZ;
      return angles;
}
*/

// Rotation matrix around X. Angle in radiants
Matrix3d Pose::RaroundX(double RX){
      Matrix3d R;
      // Rotation around X
      R  << 1, 0     , 0      ,
            0, cos(RX), -sin(RX),
            0, sin(RX),  cos(RX);
      return R;
}

// Rotation matrix around Y. Angle in radiants
Matrix3d Pose::RaroundY(double RY){
      Matrix3d R;
      // Rotation around Y
      R  << cos(RY) , 0, sin(RY),
            0      , 1, 0     ,
            -sin(RY), 0, cos(RY);
      return R;
}

// Rotation matrix around Z. Angle in radiants
Matrix3d Pose::RaroundZ(double RZ){
      Matrix3d R;
      // Rotation around Z
      R  << cos(RZ), -sin(RZ), 0,
            sin(RZ), cos(RZ) , 0,
            0     , 0      , 1;
      return R;
}

Vector4d Pose::RtoQ(Matrix3d R){
      // Transform rotation matrix to quaternion
      double x, y, z, w;
      float trace = R(0,0) + R(1,1) + R(2,2);
      if( trace > 0 ) {
            float s = 0.5f / sqrtf(trace+ 1.0f);
            w = 0.25f / s;
            x = ( R(2,1) - R(1,2) ) * s;
            y = ( R(0,2) - R(2,0) ) * s;
            z = ( R(1,0) - R(0,1) ) * s;
      } else {
            if ( R(0,0) > R(1,1) && R(0,0) > R(2,2) ) {
                  float s = 2.0f * sqrtf( 1.0f + R(0,0) - R(1,1) - R(2,2));
                  w = (R(2,1) - R(1,2) ) / s;
                  x = 0.25f * s;
                  y = (R(0,1) + R(1,0) ) / s;
                  z = (R(0,2) + R(2,0) ) / s;
            } else if (R(1,1) > R(2,2)) {
                  float s = 2.0f * sqrtf( 1.0f + R(1,1) - R(0,0) - R(2,2));
                  w = (R(0,2) - R(2,0) ) / s;
                  x = (R(0,1) + R(1,0) ) / s;
                  y = 0.25f * s;
                  z = (R(1,2) + R(2,1) ) / s;
            } else {
                  float s = 2.0f * sqrtf( 1.0f + R(2,2) - R(0,0) - R(1,1) );
                  w = (R(1,0) - R(0,1) ) / s;
                  x = (R(0,2) + R(2,0) ) / s;
                  y = (R(1,2) + R(2,1) ) / s;
                  z = 0.25f * s;
            }
      }
      // Copy value in a 4 dim vector
      Vector4d Q;
      Q << x, y, z, w;
      // Return vector
      return Q;
}

Matrix3d Pose::getR(){
      return this->rot.normalized().toRotationMatrix();
}

Matrix4d Pose::getT(){
      Matrix4d T;
      T <<  this->rot.normalized().toRotationMatrix(), this->pos,
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
      std::cout << "rot x: "   << this->rot.x();
      std::cout << "; y: " << this->rot.y();
      std::cout << "; z: " << this->rot.z();
      std::cout << "; w: " << this->rot.w();
      std::cout << std::endl;
}

void Pose::printABC_deg_(void){
      Array3d angles = this->rot.normalized().toRotationMatrix().eulerAngles(0,1,2) * 180.0 / M_PI;
      std::cout << std::fixed;
      std::cout << std::setprecision(3);
      std::cout << "pos x: "   << this->pos(0);
      std::cout << "; y: " << this->pos(1);
      std::cout << "; z: " << this->pos(2);
      std::cout << std::endl;
      std::cout << std::setprecision(3);
      std::cout << "rot RX: "   << angles(0);
      std::cout << "; RY: " << angles(1);
      std::cout << "; RZ: " << angles(2);
      std::cout << std::endl;
}
