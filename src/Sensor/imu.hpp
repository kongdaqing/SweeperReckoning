#ifndef IMU_H
#define IMU_H
#include "sensorType.hpp"


class IMU
{
public:
    IMU(std::string yamFile);
    ~IMU(){};
    Eigen::Vector3d GetCalibrAccData(const Eigen::Vector3d& rawAcc);
    Eigen::Vector3d GetCalibrGyroData(const Eigen::Vector3d& rawGyro);
    void SetCalibrRandomGyroBias(const Eigen::Vector3d& randGyrBias);
    void SetCalibrRandomAccBias(const Eigen::Vector3d& randAccBias);
    void SetGravity(double _gravity);
    void CalculateGyroBias(const IMUType& _imu);
    bool GetFinishGyroBiasCalculationFlg(){return  finishGyroBiasCalculationFlg;};
    double freq;
private:
Eigen::Vector3d accBias;
Eigen::Matrix3d accM;
Eigen::Vector3d gyrBias;
Eigen::Matrix3d gyrM;
Eigen::Vector3d randomAccBias;
Eigen::Vector3d randomGyroBias;
double gravity;
bool finishGyroBiasCalculationFlg;
};


#endif
