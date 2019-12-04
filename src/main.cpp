#include <iostream>
#include "Sensor/imu.hpp"
#include <opencv2/opencv.hpp>
#include "Fusion/mahonyAttEstimator.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

using namespace std;

struct IMUType{
    IMUType(){
        acc.setZero();
        gyro.setZero();
        time_s = 0;
    }
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    double time_s;
};

struct GroundTruthType{
    GroundTruthType()
    {
        pos.setZero();
        q.setIdentity();
        time_s = 0;
    }
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    double time_s;
};



void ReadEurocData(ifstream& imu_fin,ifstream& gt_fin,IMUType& imu,GroundTruthType& gt)
{
    string linestr[7];
   for(int i =0;i<7;i++)
   {
       if(i < 6)
       {
           getline(imu_fin,linestr[i],',');
       }else {
           getline(imu_fin,linestr[i]);
       }
       if(i == 0)
           imu.time_s =atof(linestr[i].c_str())*1e-9;
       else if ( i < 4) {
           imu.gyro[i-1] = atof(linestr[i].c_str());
       }else {
           imu.acc[i-4] = atof(linestr[i].c_str());
       }
   }
   //ground truth
   string linestr2[9];
   double q[4];
   for(int i=0; i<9; i++)
   {
       if(i < 8)
       {
           getline(gt_fin,linestr2[i],',');
       }else {
           getline(gt_fin,linestr2[i]);
       }
       if(i == 0)
           gt.time_s =atof(linestr2[i].c_str())*1e-9;
       else if ( i < 4) {
           gt.pos[i-1] = atof(linestr2[i].c_str());
       }else if( i < 8){
           q[i-5] = atof(linestr2[i].c_str());
       }
   }
   gt.q = Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
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
   string  imuDataFile =  fsSettings["data_path"];
   string  groundtruthFile = fsSettings["ground_true_path"];
   double imu_freq = fsSettings["imu_freqency"];
   printf("[Main]:IMU data path is %s\n",imuDataFile.c_str());
   printf("[Main]:GroundTruth data path is %s\n",groundtruthFile.c_str());

   MahonyAttEstimator attEstimator(configFile);
   ifstream imu_fin(imuDataFile.c_str());
   ifstream gt_fin(groundtruthFile.c_str());
   if(!imu_fin)
   {
       printf("[Main]:File %s doesn't exist!\n",imuDataFile.c_str());
       return -1;
   }
   if(!gt_fin)
   {
       printf("[Main]:File %s doesn't exist!\n",groundtruthFile.c_str());
       return -1;
   }
   string linetmp;
   getline(imu_fin,linetmp);
   getline(gt_fin,linetmp);
   unsigned int intervalTime_ms = 1000/( unsigned int)imu_freq;
   while(!imu_fin.eof())
   {
        IMUType eurocIMU;
        GroundTruthType eurocGT;
        ReadEurocData(imu_fin,gt_fin,eurocIMU,eurocGT);
        cout << "[Main]:" << eurocIMU.time_s << " " << eurocIMU.gyro.transpose() << " " << eurocIMU.acc.transpose() << "  " << eurocIMU.gyro.norm()*RAD2DEG << endl;
        if(!attEstimator.initialSuccessFlg)
        {
            attEstimator.InitializeEstimator(eurocIMU.acc,eurocIMU.gyro);
        }else {
            if(attEstimator.GetEstimatorState())
            {
                attEstimator.EstimateAttitude(eurocIMU.acc,eurocIMU.gyro,eurocIMU.time_s);
            }
        }
        chrono::milliseconds dura(intervalTime_ms);
        this_thread::sleep_for(dura);
   }

}
