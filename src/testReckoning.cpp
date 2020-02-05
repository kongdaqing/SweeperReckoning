#include <iostream>
#include "Sensor/imu.hpp"
#include <opencv2/opencv.hpp>
#include "Fusion/mahonyAttEstimator.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include "Anker/readAnkerDataFile.h"
#include "Sensor/odometry.hpp"
#include "Sensor/optFlow.hpp"
#include "AnomalyDetector/anomalyDetector.hpp"
#include "Reckoning/DeadReckoning.h"

using namespace std;

void ExtractAnkerData(AnkerData& _ankerData,IMUType& _imuData,OdometryType& _odomData,OptFlowType& _optData)
{
  _imuData.time_s = _ankerData.time;
  _imuData.acc = Eigen::Vector3d(_ankerData.ax,_ankerData.ay,_ankerData.az);
  _imuData.gyro = Eigen::Vector3d(_ankerData.gx,_ankerData.gy,_ankerData.gz);
  _odomData.time_s = _ankerData.time;
  _odomData.leftPos = _ankerData.odo_left_pos;
  _odomData.rightPos = _ankerData.odo_right_pos;
  _odomData.leftVelocity = _ankerData.odo_left_vel;
  _odomData.rightVelocity = _ankerData.odo_right_vel;
  _optData.time_s = _ankerData.time;
  _optData.optSumX = _ankerData.opt_pos_x;
  _optData.optSumY = _ankerData.opt_pos_y;
  _optData.optFigIq = _ankerData.opt_quality;

}


int main(int argc,char **argv)
{
  cout << "Hello Anker!" << endl;
  if(argc < 2)
  {
    cout << "[Error]:Please input config file!\n" << endl;
    return -1;
  }
  string configFile = argv[1];
  cv::FileStorage fsSettings(configFile,cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  string  dataPath =  fsSettings["data_path"];
  double fastSkew = fsSettings["speed_skew"];
  double imu_freq = fsSettings["imu_freqency"];
  unsigned int intervalTime_ms = 1000/( unsigned int)(fastSkew * imu_freq);
  ReadAnkerDataFile AnkerDatas(dataPath);
  IMU *imu = new IMU(configFile);
  Odometry *odom = new Odometry(configFile);
  OptFlow *opt = new OptFlow(configFile);
  AnomalyDetector anomalyDetector(configFile,imu,odom,opt);
  DeadReckoning reckoning(configFile,imu);
  OptFlowType lastOptData;
  while(AnkerDatas.AnkerDataSet.size() > 1)
  {
    AnkerData ankerData = AnkerDatas.AnkerDataSet.front();
    AnkerDatas.AnkerDataSet.pop();
    IMUType imuData;
    OdometryType odomData;
    OptFlowType optData;
    ExtractAnkerData(ankerData,imuData,odomData,optData);
    if(!imu->GetFinishGyroBiasCalculationFlg())
    {
      lastOptData = optData;
      imu->CalculateGyroBias(imuData);
    }else {
      anomalyDetector.DetectOdomOptState(imuData,optData,odomData);
      OdometryOptflowType odomOptflowData(imuData.time_s);

      int robotStatus = anomalyDetector.GetRobotState();
      switch (robotStatus) {
      case DetectWrongType::good:
          odomOptflowData.odometryBadFlg = false;
        break;
      case DetectWrongType::wheelNoTwistSlip:
          odomOptflowData.odometryBadFlg = true;
        break;
      case DetectWrongType::wheelTwistSlip:
          odomOptflowData.odometryBadFlg = true;
        break;
      case DetectWrongType::optflowFaliure:
          odomOptflowData.odometryBadFlg = false;
        break;
      default:
        break;
      }
      Eigen::Vector3d gyroCalibr = imu->GetCalibrGyroData(imuData.gyro);
      IMUType imuCalibr = imuData;
      imuCalibr.gyro = gyroCalibr;
      odom->GetOdometryVel(odomData,odomOptflowData);
      opt->GetOdoVelFromOpt(optData,lastOptData,imuCalibr,odomOptflowData);
      lastOptData = optData;
      reckoning.Update(odomOptflowData,imuData);
    }


    chrono::milliseconds dura(intervalTime_ms);
    this_thread::sleep_for(dura);
  }
  delete imu;
  delete opt;
  delete odom;
}
