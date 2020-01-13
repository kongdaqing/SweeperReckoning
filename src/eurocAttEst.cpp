#include <iostream>
#include "Sensor/imu.hpp"
#include <opencv2/opencv.hpp>
#include "Fusion/mahonyAttEstimator.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include "Euroc/readEurocData.h"

using namespace std;



struct GroundTruthType{
    GroundTruthType()
    {
        pos.setZero();
        q.setIdentity();
        time_s = 0;
    }
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    Eigen::Vector3d euler;
    double  time_s;
};

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
    string  eurocDataFile =  fsSettings["data_path"];
    printf("[Main]:Euroc data path is %s!\n",eurocDataFile.c_str());

    MahonyAttEstimator attEstimator(configFile);
    readEurocData eurocDataSet(eurocDataFile);



    int i = 0,j = 0;
    bool initialFlg = true;
    while(i < eurocDataSet.eurocImuVec.size())
    {
        eurocImu imu = eurocDataSet.eurocImuVec[i];
        if(initialFlg)
        {
            eurocEstGroundTruth gtData = eurocDataSet.eurocGroundTruthVec[j];
            if(fabs(gtData.time_s - imu.time_s) < 2/attEstimator.GetIMUDataFreqency())
            {
                attEstimator.SetQuadnion(gtData.q);
                initialFlg = false;
            }else if (gtData.time_s > imu.time_s) {
                i++;
            }else {
                j++;
            }
        }else{
            i++;
            IMUType imudata;
            imudata.time_s = imu.time_s;
            imudata.acc = imu.acc;
            imudata.gyro = imu.gyro;
            attEstimator.EstimateAttitude(imudata);
        }
        chrono::microseconds dura(10);
        this_thread::sleep_for(dura);
    }

}
