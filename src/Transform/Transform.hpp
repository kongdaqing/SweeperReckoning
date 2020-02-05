#ifndef TRANSFORM_H
#define TRANSFORM_H
#include <Eigen/Dense>


class Transform2D
{
public:
  Transform2D()
  {
     setZero();
  }
  Transform2D(Eigen::Vector3d _pose2D)
  {
    x = _pose2D[0];
    y = _pose2D[1];
    theta = _pose2D[2];
  }
  void setZero()
  {
    x = 0;
    y = 0;
    theta = 0;
  }
  void SetTransform(const Transform2D& _t)
  {
      x = _t.x;
      y = _t.y;
      theta = _t.theta;
  }
  Eigen::Vector2d Rotate(Eigen::Vector2d& vec)
  {
      Eigen::Vector2d outVec;
      outVec[0] = vec[0] * cos(theta) - vec[1] * sin(theta);
      outVec[1] = vec[0] * sin(theta) + vec[1] * cos(theta);
      return  outVec;
  }
  void UpdatePose(const Eigen::Vector2d& _pose)
  {
     x += _pose[0];
     y += _pose[1];
  }
  void UpdateTheta(double _dTheta)
  {
    theta += _dTheta;

    if(theta > M_PI)
      theta -= 2*M_PI;
    if(theta < -M_PI)
      theta += 2*M_PI;

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
