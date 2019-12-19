#include "anomalyDetector.hpp"

AnomalyDetector::AnomalyDetector(string _configFile):configFile(_configFile)
{
  Initialize();
  imuPtr = new IMU(_configFile);
  odomPtr = new Odometry(_configFile);
  optPtr = new OptFlow(_configFile);
}

AnomalyDetector::AnomalyDetector(string _configFile,IMU *_imu,Odometry* _odom,OptFlow *_opt)
  :configFile(_configFile),odomPtr(_odom),optPtr(_opt),imuPtr(_imu)
{
  Initialize();
}


AnomalyDetector::~AnomalyDetector()
{
  /*
  if(imuPtr != nullptr)
    delete  imuPtr;
  if(odomPtr != nullptr)
    delete odomPtr;
  if(optPtr != nullptr)
    delete optPtr;*/
  recordFile.close();
}

void AnomalyDetector::Initialize()
{
  cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  string recordPath = fsSettings["data_path"];
  detectPeriodNum = fsSettings["detect_count"];
  detectDPosThreshold = fsSettings["detect_thr"];

  if(  detectPeriodNum < 1)
    detectPeriodNum = 1;
  if(detectDPosThreshold < 0.005)
    detectDPosThreshold = 0.005;
  recordPath = recordPath + "output/anomaly_detect.csv";

  getGyroBiasFlg = false;
  recordFile.open(recordPath);
  recordFile << "Timestamp" << "," << "odo_px" << "," << "odo_py" << "," << "odo_theta" << "," << "opt_px" << ","
             << "opt_py" << "," << "opt_theta" << "," << "dPx" << "," << "dPy" << "," << "dTheta" << "," << "dNorm" << "," << "state" <<  endl;
}



void AnomalyDetector::DetectOdomOptState(const IMUType &_imu, const OptFlowType &_opt, const OdometryType &_odom)
{
  static OptFlowType lastOpt = _opt;
  static OdometryType lastOdom = _odom;
  if(!imuPtr->GetFinishGyroBiasCalculationFlg())
  {
    return;
  }
  Eigen::Vector3d calibrGyro = imuPtr->GetCalibrGyroData(_imu.gyro);
  IMUType imuCalibr = _imu;
  imuCalibr.gyro = calibrGyro;

  optPtr->CheckOptflowDataQuality(_opt);
  Eigen::Vector3d dposFopt = optPtr->TransformOpt2Odom(_opt,lastOpt,imuCalibr);
  lastOpt = _opt;
  Eigen::Vector3d dposFodo = odomPtr->GetOdometryDeltaPose(_odom,lastOdom);
  lastOdom = _odom;
  detectFactor.count++;
  detectFactor.posFopt += dposFopt;
  detectFactor.posFodom += dposFodo;
  if(detectFactor.count == detectPeriodNum)
  {
    detectFactor.posFopt[2] = 0;
    detectFactor.posFodom[2] = 0;
    Eigen::Vector3d diffPos = detectFactor.posFodom - detectFactor.posFopt;
    double posNorm = diffPos.norm();
    if(posNorm > detectDPosThreshold)
    {
       bool optDataGoodQualityFlg = optPtr->GetOptflowDataQualityFlg();
      if(detectFactor.posFopt.norm() < detectFactor.posFodom.norm())
      {
        //KDQ-NOTE:
        //This is happened when wheel is slipping or optflow is failure.
        if(optDataGoodQualityFlg)
           detectFactor.robotStatus = DetectWrongType::wheelTwistSlip;
        else
           detectFactor.robotStatus = DetectWrongType::optflowFaliure;
      }else{
        if(optDataGoodQualityFlg)
           detectFactor.robotStatus = DetectWrongType::wheelNoTwistSlip;
        else
           detectFactor.robotStatus = DetectWrongType::optflowFaliure;
      }
    }else {
        detectFactor.robotStatus = DetectWrongType::good;
    }
    recordFile << _imu.time_s << "," << detectFactor.posFodom[0] << "," << detectFactor.posFodom[1] << "," << detectFactor.posFodom[2] << ","
                              << detectFactor.posFopt[0] << "," << detectFactor.posFopt[1] << "," << detectFactor.posFopt[2] << ","
                              << diffPos[0] << "," << diffPos[1] << "," << diffPos[2]  << "," << posNorm << ","<< detectFactor.robotStatus << endl;
    printf("[AnomalyDetector]:Timestamp %f diffDpos %.5f %.5f %.5f\n",_imu.time_s,diffPos[0],diffPos[1],diffPos[2]);
    detectFactor.Clear();
  }


}
