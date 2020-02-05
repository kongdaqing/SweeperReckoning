#ifndef SENSORTYPE_H
#define SENSORTYPE_H
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#define STEADY_GYRO_MINIRATE 2
#define RAD2DEG 57.29578
#define INITIALCOUNT 1000
#define G 9.7936

struct OptFlowType
{
    OptFlowType()
    {
        time_s = 0;
        optSumX = 0;
        optSumY = 0;
        optFigIq = 0;
    }
    double time_s;
    double optSumX;
    double optSumY;
    unsigned short int optFigIq;

};


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

struct OdometryOptflowType
{
  OdometryOptflowType()
  {
    time_s = 0;
    odomVel.setZero();
    optVel.setZero();
    odometryBadFlg = false;
  }
  OdometryOptflowType(double _time_s)
  {
    time_s = _time_s;
    odomVel.setZero();
    optVel.setZero();
    odometryBadFlg = false;
  }
  Eigen::Vector2d odomVel;
  Eigen::Vector2d optVel;
  bool odometryBadFlg;
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
