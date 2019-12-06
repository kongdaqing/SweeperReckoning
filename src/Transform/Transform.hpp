#ifndef TRANSFORM_H
#define TRANSFORM_H
#include <Eigen/Dense>
#define pi 3.1415926

class Transform2D
{
public:
  Transform2D()
  {
     setZero();
  }
  void setZero()
  {
    x = 0;
    y = 0;
    theta = 0;
  }
  void SetTransform(Transform2D& _t)
  {
      x = _t.x;
      y = _t.y;
      theta = _t.theta;
  }
  void UpdateTheta(double _dTheta)
  {
    theta += _dTheta;
    if(theta > pi)
      theta -= 2*pi;
    if(theta < -pi)
      theta += 2*pi;
  }
  double x;
  double y;
  double theta;
};

class Transform3D
{
public:
  Transform3D()
  {
	pos.setZero();
        q.setIdentity();
  }
Eigen::Vector3d pos;
Eigen::Quaterniond q;

};

#endif
