#include "optFlow.hpp"

OptFlow::OptFlow(string _configFile)
{
  cv::FileStorage fsSetting(_configFile,cv::FileStorage::READ);
  string savePath = fsSetting["data_path"];
  savePath += "output/optT.csv";
  freq = fsSetting["opt_freqency"];
  cv::Mat cv_T;
  fsSetting["opt_T_odom"] >> cv_T;
  cv::cv2eigen(cv_T, T);
  printf("[Optflow]:Extrisic from Optflow to odometry is T \n");
  cout << T << endl;
  optIqThrMax = fsSetting["opt_iqThr"];
  optIqTimeMin = fsSetting["opt_iqTime"];
  if(optIqThrMax < 1300)
    optIqThrMax = 1300;
  if(optIqTimeMin < 0.1)
    optIqTimeMin = 0.1;
  optflowGoodQualityFlg = true;
}



Eigen::Vector3d OptFlow::TransformOpt2Odom(const OptFlowType &_opt,const OptFlowType & _lastOpt,const IMUType& _imu)
{

  double dt = _opt.time_s - _lastOpt.time_s;
  if(dt <= 0 || dt > 4/freq)
  {
    printf("[Optflow]:Timestamp is not right curTime: %f and lastTime:%f\n",_opt.time_s,_lastOpt.time_s);
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d optData,odoData;
  optData[0] = _opt.optSumX - _lastOpt.optSumX;
  optData[1] = _opt.optSumY - _lastOpt.optSumY;
  optData[2] = _imu.gyro.z() * dt;
  odoData = T*optData;
  //cout << "[Optflow]: odomData is " << odoData.transpose() << endl;
  return  odoData;
}

void OptFlow::GetOdoVelFromOpt(const OptFlowType &_opt,const OptFlowType & _lastOpt,const IMUType &_imu,OdometryOptflowType& odomOptData)
{
  double dt = _opt.time_s - _lastOpt.time_s;
  if(dt > 4/freq || dt <= 0)
  {
    printf("[Optflow]:Timestamp is not right between curTime %f and lastTime %f\n",_opt.time_s,_lastOpt.time_s);
    return;
  }
  Eigen::Vector3d dpos;
  dpos = TransformOpt2Odom(_opt,_lastOpt,_imu);
  odomOptData.optVel[0] = dpos[0]/dt;
  odomOptData.optVel[1] = dpos[1]/dt;

}



void OptFlow::CheckOptflowDataQuality(const OptFlowType &_opt)
{
  static int count[2] = {0};
  if(_opt.optFigIq > optIqThrMax)
  {
    if(optflowGoodQualityFlg)
    {
      count[0]++;
      if(count[0] > optIqTimeMin*freq)
      {
        count[0] = 0;

        optflowGoodQualityFlg = false;
      }
    }
    count[1] = 0;
  }else {
    if(!optflowGoodQualityFlg)
    {
      count[1]++;
      if(count[1] > optIqTimeMin*freq)
      {
        count[1] = 0;
        optflowGoodQualityFlg = true;
      }
    }
    count[0] = 0;
  }

}
