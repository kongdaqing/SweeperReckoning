#include "imu.hpp"

IMU::IMU(std::string yamlFile)
{
  accM.setIdentity();
  accBias.setZero();
  gyrM.setIdentity();
  gyrBias.setZero();
  randomAccBias.setZero();
  randomGyroBias.setZero();
  gravity = G;
  finishGyroBiasCalculationFlg = false;
  cv::FileStorage fsSettings(yamlFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  cv::Mat cv_T;
  fsSettings["acc_T_calibr"] >> cv_T;
  Eigen::Matrix4d T;
  cv::cv2eigen(cv_T, T);
  accM = T.block<3, 3>(0, 0);
  accBias = T.block<3, 1>(0, 3);
  std::cout <<"[IMU]:\n" << "accM: \n" << accM << "\naccBias: " << accBias.transpose() << std::endl;
  freq = fsSettings["imu_freqency"];
  std::cout << "IMU Freqency: " << freq << std::endl;
}
Eigen::Vector3d IMU::GetCalibrAccData(const Eigen::Vector3d &rawAcc)
{
    Eigen::Vector3d res;
    res = accM * (rawAcc + accBias) - randomAccBias;
    return res;
}

Eigen::Vector3d IMU::GetCalibrGyroData(const Eigen::Vector3d &rawGyro)
{
    Eigen::Vector3d res;
    res = gyrM * (rawGyro + gyrBias) - randomGyroBias;
    return res;
}


void IMU::SetCalibrRandomAccBias(const Eigen::Vector3d &randAccBias)
{
    randomAccBias = randAccBias;
    printf("[IMU]:Set random acc bias = %f %f %f\n",randomAccBias[0],randomAccBias[1],randomAccBias[2]);
}

void IMU::SetCalibrRandomGyroBias(const Eigen::Vector3d &randGyrBias)
{
    randomGyroBias = randGyrBias;
    printf("[IMU]:Set random gyro bias = %f %f %f\n",randomGyroBias[0],randomGyroBias[1],randomGyroBias[2]);
}

void IMU::SetGravity(double _gravity)
{
    gravity = _gravity;
}

void IMU::CalculateGyroBias(const IMUType &_imu)
{
  static Eigen::Vector3d sumGyro{Eigen::Vector3d::Zero()};
  static int countSum = 0;
  if(finishGyroBiasCalculationFlg)
      return;
  Eigen::Vector3d resAcc;
  Eigen::Vector3d resGyr;
  resGyr = GetCalibrGyroData(_imu.gyro);
  if(resGyr.norm()*RAD2DEG > STEADY_GYRO_MINIRATE)
  {
      printf("[Estimator]:Please keep robot steay for Initialization!!!\n");
      sumGyro.setZero();
      countSum = 0;
      return;
  }
  countSum++;
  sumGyro += resGyr;
  if(countSum == INITIALCOUNT)
  {

      Eigen::Vector3d averGyro = sumGyro/INITIALCOUNT;
      SetCalibrRandomGyroBias(averGyro);
      finishGyroBiasCalculationFlg = true;
  }
}
