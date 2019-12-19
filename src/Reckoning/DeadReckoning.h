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
  void Initialize();
  void SetPose(Transform2D _pose);

private:
  Transform2D pose2D;
  IMU *imu;
  bool finishInitializationFlg;
  OdometryOptData lastData;
  std::ofstream recordFile;
};

#endif // DEADRECKONING_H
