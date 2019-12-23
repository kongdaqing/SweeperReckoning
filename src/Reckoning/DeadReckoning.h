#ifndef DEADRECKONING_H
#define DEADRECKONING_H
#include "../Sensor/imu.hpp"
#include "../Transform/Transform.hpp"
struct OdometryOptData
{
    OdometryOptData()
    {
        firstFlg = true;
        Spd.setZero();
    }
    OdometryOptflowType odomOpt;
    IMUType imu;
    Eigen::Vector2d Spd;
    bool firstFlg;
};
class DeadReckoning
{
public:
  DeadReckoning(std::string _configFile,IMU *_imu);
  ~DeadReckoning();
  void Update(const OdometryOptflowType& _odomOptData,const IMUType& _imu);
  void Transform2DMidUpdate(Transform2D& _T2D,const Eigen::Vector2d& _lastVel,const Eigen::Vector2d& _curVel,double _lastGz,double _curGz,double dt);
  void Initialize();
  void SetPose(Transform2D _pose);


private:
  Transform2D pose2D,odoPose2D,optPose2D;
  IMU *imu;
  bool finishInitializationFlg;
  OdometryOptData lastData;
  std::ofstream recordFile;
};

#endif // DEADRECKONING_H
