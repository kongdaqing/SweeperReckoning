#include "optFlow.hpp"
#include <opencv2/opencv.hpp>
#include "../Utility/utility.hpp"
#include <opencv2/core/eigen.hpp>
OptFlow::OptFlow(string _configFile)
{
  cv::FileStorage fsSetting(_configFile,cv::FileStorage::READ);
  string savePath = fsSetting["data_path"];
  savePath += "output/optT.csv";
  recordFile.open(savePath);
  recordFile << "time_s" << "," << "dpx" << "," << "dpy" << "," << "px" << "," << "py" << "," <<  "theta" << endl;
  freq = fsSetting["opt_freqency"];
  cv::Mat cv_T;
  fsSetting["opt_T_odom"] >> cv_T;
  cv::cv2eigen(cv_T, T);
  printf("[Optflow]:Extrisic from Optflow to odometry is T \n");
  cout << T << endl;
  lastData.time_s = -1.;
  T_opt.setZero();
  goodStateFlg = true;
}



Eigen::Vector3d OptFlow::TransformOpt2Odom(const OptFlowType &_opt,const IMUType& _imu)
{
  Eigen::Vector3d optData;
  Eigen::Vector3d odoData;
  double dt = _opt.time_s - lastData.time_s;
  optData[0] = _opt.optSumX - lastData.optSumX;
  optData[1] = _opt.optSumY - lastData.optSumY;
  optData[2] = _imu.gyro.z() * dt;
  odoData = T*optData;
  return  odoData;
}

void OptFlow::DeadReckoningUpdate(const OptFlowType &_opt, const IMUType &_imu)
{
  double dt = _opt.time_s - lastData.time_s;
  double dTime = _opt.time_s - _imu.time_s;
  if(lastData.time_s < 0 )
  {
    lastData = _opt;
    T_opt.setZero();
    return;
  }

  if(dt > 10./freq || dt < 0 || fabs(dTime) > 1./freq)
  {
    goodStateFlg = false;
    printf("[Optflow]:Please check optflow data timestamp,its interval is so big!\n");
    return;
  }
  Eigen::Vector3d dpos = TransformOpt2Odom(_opt,_imu);
  lastData = _opt;
  T_opt.x += dpos[0] * cos(T_opt.theta);
  T_opt.y += dpos[0] * sin(T_opt.theta);
  T_opt.UpdateTheta(dpos[2]);
  printf("[Optflow]:Timestamp %f pose is %f,%f,%f\n",_opt.time_s,T_opt.x,T_opt.y,T_opt.theta*RAD2DEG);
  recordFile << _opt.time_s << "," << dpos[0] << "," << dpos[1] << "," << T_opt.x << "," << T_opt.y << "," << T_opt.theta << endl;

}
