#include "DeadReckoning.h"

DeadReckoning::DeadReckoning(std::string _configFile,IMU *_imu):imu(_imu)
{
    finishInitializationFlg = false;
    pose2D.setZero();
    cv::FileStorage fsSettings(_configFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string recordPath = fsSettings["data_path"];
    recordPath = recordPath + "output/reckoning.csv";
    recordFile.open(recordPath);
    recordFile << "timestamp" << "," << "odo_vx" << "," << "odo_vy" << "," << "opt_vx" << ","
               << "opt_vy" <<  "," <<  "px" << "," << "py" << "," << "theta"  <<  "," << "flg" << std::endl;

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
    double dt = _imu.time_s - lastData.imu.time_s;
    if(dt > 4/imu->freq || dt <= 0)
    {
        printf("[Odometry]:Timestamp is not right recording curTime:%f vs lastTime:%f\n",_imu.time_s,lastData.imu.time_s);
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
    Eigen::Vector3d lastImuGyro = imu->GetCalibrGyroData(lastData.imu.gyro);
    Eigen::Vector3d curImuGyro = imu->GetCalibrGyroData(_imu.gyro);
    double gz = 0.5 * (lastImuGyro.z() + curImuGyro.z());
    Eigen::Vector2d lastDpos = lastData.Spd * dt;
    Eigen::Vector2d curDpos = curVel * dt;


    lastDpos = pose2D.Rotate(lastDpos);
    pose2D.UpdateTheta(gz * dt);
    curDpos = pose2D.Rotate(curDpos);
    curDpos = 0.5 * (curDpos + lastDpos);
   // std::cout << "[Reckoning]:curVel is " << _odomOptData.optVel[0] << "," << _odomOptData.optVel[1] << std::endl;

    pose2D.UpdatePose(curDpos);

    lastData.imu = _imu;
    lastData.odomOpt = _odomOptData;
    lastData.Spd = curVel;
    recordFile << _imu.time_s << "," << _odomOptData.odomVel[0] << "," << _odomOptData.odomVel[1] << "," << _odomOptData.optVel[0] << ","
               << _odomOptData.optVel[1] << "," << pose2D.x << "," << pose2D.y << "," << pose2D.theta << "," << _odomOptData.odometryBadFlg << std::endl;
}
