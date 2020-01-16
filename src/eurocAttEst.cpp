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
                printf("[Main]:Get first gt quad is w:%f x:%f y:%f z:%f!\n",gtData.q.w(),gtData.q.x(),gtData.q.y(),gtData.q.z());
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
            //KDQ: we can't estimate yaw value unless we have magnetometer,so yaw value is far away groudtruth gradually!
            attEstimator.EstimateAttitude(imudata);
        }
        chrono::microseconds dura(10);
        this_thread::sleep_for(dura);
    }

}
