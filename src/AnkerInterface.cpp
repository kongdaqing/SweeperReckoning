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
using namespace std;

void ExtractAnkerData(AnkerData& _ankerData,IMUType& _imuData,OdometryType& _odomData)
{
    _imuData.time_s = _ankerData.time;
    _imuData.acc = Eigen::Vector3d(_ankerData.ax,_ankerData.ay,_ankerData.az);
    _imuData.gyro = Eigen::Vector3d(_ankerData.gx,_ankerData.gy,_ankerData.gz);
    _odomData.time_s = _ankerData.time;
    _odomData.leftPos = _ankerData.odo_left_pos;
    _odomData.rightPos = _ankerData.odo_right_pos;
    _odomData.leftVelocity = _ankerData.odo_left_vel;
    _odomData.rightVelocity = _ankerData.odo_right_vel;
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
    string imu_file = dataPath +  "imu_file.cvs";
    string odo_file = dataPath + "odometer_file.cvs";
    string opt_file = dataPath + "optical_flow_file.cvs";
    double imu_freq = fsSettings["imu_freqency"];
    ReadAnkerDataFile AnkerDatas(imu_file,odo_file,opt_file);
    MahonyAttEstimator attEstimator(configFile);
    Odometry odomEstimator(configFile);

    unsigned int intervalTime_ms = 1000/( unsigned int)imu_freq;

    while(AnkerDatas.AnkerDataSet.size() > 1)
    {
        AnkerData ankerData = AnkerDatas.AnkerDataSet.front();
        AnkerDatas.AnkerDataSet.pop();
        IMUType imuData;
        OdometryType odomData;
        ExtractAnkerData(ankerData,imuData,odomData);
        if(!attEstimator.initialSuccessFlg)
        {
            attEstimator.InitializeEstimator(imuData);
        }else {
            Eigen::Vector3d Rate_b = attEstimator.imu->GetCalibrGyroData(imuData.gyro);
            if(attEstimator.GetEstimatorState())
            {
                attEstimator.EstimateAttitude(imuData);
                if(odomEstimator.GetDeadReckoningState())
                {
                  Eigen::Quaterniond Quad_b2w = attEstimator.GetAttQuadnion();
                  Eigen::Vector3d Rate_w = Quad_b2w * Rate_b;
                  odomEstimator.DeadReckoningUpdate(odomData,Rate_b.z());
                }
            }else {
              if(odomEstimator.GetDeadReckoningState())
              {
                odomEstimator.DeadReckoningUpdate(odomData,Rate_b.z());
              }
            }
            chrono::milliseconds dura(intervalTime_ms);
            this_thread::sleep_for(dura);
        }

    }
}
