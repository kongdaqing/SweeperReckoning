#include "DeadReckoning.h"

DeadReckoning::DeadReckoning(std::string _configFile,IMU *_imu):imu(_imu)
{
    finishInitializationFlg = false;
    pose2D.setZero();
    optPose2D.setZero();
    odoPose2D.setZero();
    cv::FileStorage fsSettings(_configFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string recordPath = fsSettings["data_path"];
    recordPath = recordPath + "output/reckoning.csv";
    recordFile.open(recordPath);
    recordFile << "timestamp" << "," << "px" << "," << "py" << "," << "theta" << ","
               << "odom_px" <<  "," << "odom_py" << "," << "odom_theta" << ","
               << "opt_px"  <<  "," << "opt_py" << "," << "opt_theta" << "," << "flg" << std::endl;

}

DeadReckoning::~DeadReckoning()
{
  recordFile.close();
 /*
  if(imu != nullptr)
    delete imu;
*/
}

void DeadReckoning::SetPose(Transform2D _pose)
{
    pose2D.SetTransform(_pose);
    odoPose2D.SetTransform(_pose);
    optPose2D.SetTransform(_pose);
}

void DeadReckoning::Initialize()
{
   if(imu->GetFinishGyroBiasCalculationFlg())
   {
       Transform2D originPos(Eigen::Vector3d::Zero());
       SetPose(originPos);
       finishInitializationFlg = true;
   }
}

void DeadReckoning::Transform2DMidUpdate(Transform2D& _T2D,const Eigen::Vector2d &_lastVel, const Eigen::Vector2d &_curVel, double _lastGz, double _curGz, double dt)
{
  Eigen::Vector2d lastDpos = _lastVel * dt;
  Eigen::Vector2d curDpos = _curVel * dt;
  lastDpos = _T2D.Rotate(lastDpos);
  double wz = 0.5 * (_lastGz + _curGz) * dt;
  _T2D.UpdateTheta(wz);
  curDpos = _T2D.Rotate(curDpos);
  curDpos = 0.5 * (lastDpos + curDpos);
  _T2D.UpdatePose(curDpos);

}
void DeadReckoning::Update(const OdometryOptflowType &_odomOptData, const IMUType &_imu)
{

    if(!finishInitializationFlg)
    {
      Initialize();
      return;
    }
    if(lastData.firstFlg)
    {
      lastData.imu = _imu;
      lastData.odomOpt = _odomOptData;
      if(_odomOptData.odometryBadFlg)
        lastData.Spd = _odomOptData.optVel;
      else
        lastData.Spd = _odomOptData.odomVel;
      lastData.firstFlg = false;
      return;
    }

    Eigen::Vector2d curVel;
    curVel.setZero();
    if(_odomOptData.odometryBadFlg)
    {
       curVel[0] = _odomOptData.optVel[0];
       curVel[1] = _odomOptData.optVel[1];
    }else {
       curVel[0] = _odomOptData.odomVel[0];
       curVel[1] = _odomOptData.odomVel[1];
    }
    double dt = _imu.time_s - lastData.imu.time_s;
    if(dt > 4/imu->freq || dt <= 0)
    {
      lastData.imu = _imu;
      lastData.odomOpt = _odomOptData;
      lastData.Spd = curVel;
      printf("[DeadReckoning]:Timestamp is not right recording curTime:%f and lastTime:%f\n",_imu.time_s,lastData.imu.time_s);
      return;
    }

    Eigen::Vector3d lastImuGyro = imu->GetCalibrGyroData(lastData.imu.gyro);
    Eigen::Vector3d curImuGyro = imu->GetCalibrGyroData(_imu.gyro);
    Transform2DMidUpdate(pose2D,lastData.Spd,curVel,lastImuGyro.z(),curImuGyro.z(),dt);
    Transform2DMidUpdate(odoPose2D,lastData.odomOpt.odomVel,_odomOptData.odomVel,lastImuGyro.z(),curImuGyro.z(),dt);
    Transform2DMidUpdate(optPose2D,lastData.odomOpt.optVel,_odomOptData.optVel,lastImuGyro.z(),curImuGyro.z(),dt);
    recordFile << _imu.time_s << "," << pose2D.x << "," << pose2D.y << "," << pose2D.theta << "," << odoPose2D.x
               << "," << odoPose2D.y << "," << odoPose2D.theta << "," << optPose2D.x << "," << optPose2D.y << ","
               << optPose2D.theta << "," << _odomOptData.odometryBadFlg << std::endl;
    lastData.imu = _imu;
    lastData.odomOpt = _odomOptData;
    lastData.Spd = curVel;
}
