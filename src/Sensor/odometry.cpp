#include "odometry.hpp"



Odometry::Odometry(string _configFile)
{
  cv::FileStorage fsSettings(_configFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  baseLine = fsSettings["odom_baseline"];
  freq = fsSettings["odom_freqency"];
  printf("[Odometry]:Wheel distance is %f\n",baseLine);
  printf("[Odometry]:OdomData freqency is %f\n",freq);

}
Odometry::~Odometry()
{

}




Eigen::Vector3d Odometry::GetOdometryDeltaPose(const OdometryType &_odoData,const OdometryType &_lastOdom)
{

    double dt = _odoData.time_s - _lastOdom.time_s;
    if(dt < 0)
    {
      printf("[Odometry]:Timestamp is not right recording curTime:%f vs lastTime:%f\n",_odoData.time_s,_lastOdom.time_s);
      return Eigen::Vector3d::Zero();
    }
    double leftVel = 0.5 * (_odoData.leftVelocity + _lastOdom.leftVelocity);
    double rightVel = 0.5 * (_odoData.rightVelocity + _lastOdom.rightVelocity);
    double dPosx = 0.5 * dt * (leftVel + rightVel);
    double dTheta = dt * (rightVel - leftVel)/baseLine;
    Eigen::Vector3d dPos(dPosx,0,dTheta);
    return dPos;
}

void Odometry::GetOdometryVel(const OdometryType &_odoData, OdometryOptflowType &odomOptData)
{
  odomOptData.odomVel[0] = 0.5 * (_odoData.leftVelocity + _odoData.rightVelocity);
  odomOptData.odomVel[1] = 0;
}
