#ifndef SENSORTYPE_H
#define SENSORTYPE_H
#include <Eigen/Dense>


struct IMUType{
    IMUType(){
        acc.setZero();
        gyro.setZero();
        time_s = 0;
    }
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    double time_s;
};

struct OdometryType
{
  OdometryType()
  {
    time_s = 0;
    leftVelocity = 0;
    rightVelocity = 0;
    leftPos = 0;
    rightPos = 0;
  }
  double time_s;
  double leftVelocity;
  double rightVelocity;
  double leftPos;
  double rightPos;
};

#endif
