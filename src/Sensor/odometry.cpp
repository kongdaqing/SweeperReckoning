#include "odometry.hpp"
#include "opencv2/opencv.hpp"


Odometry::Odometry(string _configFile)
{
  cv::FileStorage fsSettings(_configFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  baseLine = fsSettings["odom_baseline"];
  freq = fsSettings["odom_freqency"];
  string recordPath = fsSettings["data_path"];
  recordPath = recordPath + "output/pose2D.csv";
  printf("[Odometry]:Wheel distance is %f\n",baseLine);
  printf("[Odometry]:OdomData freqency is %f\n",freq);
  printf("[Odometry]:Record transform file is %s\n",recordPath.c_str());
  recordFile.open(recordPath);
  recordFile.precision(9);
  recordFile << "t" << "," << "x" << "," << "y" << "," << "theta" << "," << "x_i" << "," << "y_i" << "," << "theta_i" <<  endl;
  initialFlg = false;
  goodStateFlg = true;
  badTimestampCount = 0;
  T.setZero();
  T_imu.setZero();
}
Odometry::~Odometry()
{

}
const Transform2D  Odometry::GetTransform2D()
{
  return T;
}
void Odometry::SetTransform(const Transform2D &_T)
{
  T.SetTransform(_T);
}

void Odometry::InitializeDeadReckoning(const OdometryType &_odoData)
{
  if(fabs(_odoData.leftVelocity) > 1e-6 || fabs(_odoData.rightVelocity) > 1e-6)
  {
    printf("[Odometry]:Please keep robot no move for initializing Odometry Dead Reckoning system!\n");
    return;
  }
  initialFlg = true;
  lastTime_s = _odoData.time_s;
  badTimestampCount = 0;
  T.setZero();
  T_imu.setZero();
}

void Odometry::DeadReckoningUpdate(const OdometryType &_odoData,const IMUType& _imuData)
{
  if(!initialFlg)
  {
      InitializeDeadReckoning(_odoData);
      return;
  }
  double dT = _odoData.time_s - lastTime_s;
  lastTime_s = _odoData.time_s;

  if(dT < 0)
  {
    badTimestampCount++;
    printf("[Odometry]:Timestamp is not right,please input right timestamp!\n");
    return;
  }
  if(dT > 10./freq)
  {
    badTimestampCount += 10;
    printf("[Odometry]:Timestamp interval is too big,please input right timestamp!\n");
    return;
  }

  double dAlignTime = _odoData.time_s - _imuData.time_s;
  if(badTimestampCount > 100 || fabs(dAlignTime) > 1./freq)
  {
    goodStateFlg = false;
    return;
  }
  double dPos = 0.5 * dT * (_odoData.leftVelocity + _odoData.rightVelocity);
  T.x += dPos * cos(T.theta);
  T.y += dPos * sin(T.theta);
  double dTheta = dT * (_odoData.rightVelocity - _odoData.leftVelocity)/baseLine;
  T.UpdateTheta(dTheta);

  T_imu.x += dPos * cos(T_imu.theta);
  T_imu.y += dPos * sin(T_imu.theta);
  T_imu.UpdateTheta(_imuData.gyro.z()*dT);
  //printf("[Odometry]:Timestamp is %f, odom data %f %f\n",_odoData.time_s,_odoData.rightVelocity,_odoData.leftVelocity);
  printf("[Odometry]:Dead Rockoning x,y,theta is %f,%f,%f\n",T.x,T.y,T.theta*RAD2DEG);
  printf("[Odometry]:IMU DRockoning x,y,theta is %f,%f,%f\n",T_imu.x,T_imu.y,T_imu.theta*RAD2DEG);
  double time_left_s = fmod(_odoData.time_s,10000);
  recordFile << time_left_s << "," << T.x << "," << T.y << "," << T.theta << "," << T_imu.x << "," << T_imu.y << "," << T_imu.theta << endl;
}

