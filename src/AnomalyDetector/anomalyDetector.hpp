#ifndef ANOMALYDETECTOR_H
#define ANOMALYDETECTOR_H
#include "../Sensor/sensorType.hpp"
#include "../Sensor/imu.hpp"
#include "../Sensor/odometry.hpp"
#include "../Sensor/optFlow.hpp"
#include <unistd.h>
#include <fstream>
using namespace std;

enum DetectWrongType
{
  good,
  wheelNoTwistSlip,
  wheelTwistSlip,
  optflowFaliure
};
struct DetectType
{
  DetectType()
  {
    Clear();
    robotStatus =DetectWrongType::good;
  };
  void Clear()
  {
    posFopt.setZero();
    posFodom.setZero();
    count = 0;
  };
  Eigen::Vector3d posFodom;
  Eigen::Vector3d posFopt;
  int count;
  int robotStatus;
};

class AnomalyDetector{
public:
AnomalyDetector(string _configFile);
AnomalyDetector(string _configFile,IMU *_imu,Odometry* _odom,OptFlow *_opt);
void Initialize();
~AnomalyDetector();
void DetectOdomOptState(const IMUType& _imu,const OptFlowType& _opt,const OdometryType& _odom);
int GetRobotState(){return detectFactor.robotStatus;};
private:
bool getGyroBiasFlg;
string configFile;
ofstream recordFile;
Odometry * odomPtr;
OptFlow * optPtr;
IMU * imuPtr;
int detectPeriodNum;
double detectDPosThreshold;
DetectType detectFactor;


};

#endif
