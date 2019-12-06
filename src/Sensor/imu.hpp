#ifndef IMU_H
#define IMU_H
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#define G 9.7936



class IMU
{
public:
    IMU(std::string yamFile);
    ~IMU(){};
    Eigen::Vector3d GetCalibrAccData(Eigen::Vector3d& rawAcc);
    Eigen::Vector3d GetCalibrGyroData(Eigen::Vector3d& rawGyro);
    void SetCalibrRandomGyroBias(Eigen::Vector3d& randGyrBias);
    void SetCalibrRandomAccBias(Eigen::Vector3d& randAccBias);
    double freq;
private:
Eigen::Vector3d accBias;
Eigen::Matrix3d accM;
Eigen::Vector3d gyrBias;
Eigen::Matrix3d gyrM;
Eigen::Vector3d randomAccBias;
Eigen::Vector3d randomGyroBias;
};


#endif
