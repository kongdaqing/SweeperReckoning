#include "readEurocData.h"
#include "../Utility/utility.hpp"
readEurocData::readEurocData(const string& _dataPath)
{
    string imuPath = _dataPath + "imu0/data.csv";
    string gtPath = _dataPath + "state_groundtruth_estimate0/data.csv";
    string vcPosePath = _dataPath + "leica0/data.csv";
    string saveAttPath = _dataPath + "gt_attitude.csv";
    attRecord.open(saveAttPath.c_str());
    attRecord << fixed;
    attRecord << "#" << "t" << "," << "qw" << "," << "qx" << "," << "qy" << "," << "qz" << "," << "roll" << "," << "pitch" << "," << "yaw"  << endl;

    readIMUData(imuPath);
    readEstGroundTruth(gtPath);
    readVisualCapturePosition(vcPosePath);
    printf("[ReadEurocData]:Read Finish!IMUData size is %d,Estimate GroundTrut size is %d and VisualCapture"
           " Pose size is %d!\n",eurocImuVec.size(),eurocGroundTruthVec.size(),eurocVCPosVec.size());
    attRecord.close();
}

void readEurocData::readIMUData(const string &_imuPath)
{
    ifstream imuFile(_imuPath.c_str());
    if(!imuFile)
    {
        printf("[ReadEurocData]:Path %s is not exsit!Please input right imu file path!\n",_imuPath.c_str());
    }
    string firstline;
    getline(imuFile,firstline);
    while(!imuFile.eof())
    {
        string lineStr[7];
        long int time;
        Vector3d gyro,acc;
        for(int i =0;i<7;i++)
        {
            if(i < 6)
            {
                getline(imuFile,lineStr[i],',');
            }else {
                getline(imuFile,lineStr[i]);
            }
            if(i == 0)
                time =atol(lineStr[i].c_str());
            else if ( i < 4) {
                gyro[i-1] = atof(lineStr[i].c_str());
            }else {
                acc[i-4] = atof(lineStr[i].c_str());
            }
        }
        if(!lineStr->empty())
        {
            eurocImu imuData(time,gyro,acc);
            eurocImuVec.push_back(imuData);
            if(eurocImuVec.size() % 200 == 0)
                printf("[ReadEurocData]:Euroc imu current time_s is %f and recive size is %d!\n",imuData.time_s,eurocImuVec.size());
        }

    }
}

void readEurocData::readEstGroundTruth(const string &_gtPath)
{
    ifstream gtFile(_gtPath.c_str());
    if(!gtFile)
    {
        printf("[ReadEurocData]:Path %s is not exsit!Please input right imu file path!\n",_gtPath.c_str());
        return;
    }
    string firstline;
    getline(gtFile,firstline);
    while(!gtFile.eof())
    {
        string lineStr[17];
        long int time;
        double quat[4];
        Vector3d pos;
        Vector3d vel;
        Vector3d bias_w;
        Vector3d bias_a;
        for(int i =0;i<14;i++)
        {
            if(i < 13)
            {
                getline(gtFile,lineStr[i],',');
            }else {
                getline(gtFile,lineStr[i]);
            }
            if(i == 0)
                time =atol(lineStr[i].c_str());
            else if ( i < 4) {
                pos[i-1] = atof(lineStr[i].c_str());
            }else if(i < 8){
                quat[i-4] = atof(lineStr[i].c_str());
            }else if(i < 11){
                vel[i-8] = atof(lineStr[i].c_str());
            }else if(i < 14){
                bias_w[i-11] = atof(lineStr[i].c_str());
            }else {
                bias_a[i-14] = atof(lineStr[i].c_str());
            }

        }
        if(!lineStr->empty())
        {
            Quaterniond q(quat[0],quat[1],quat[2],quat[3]);
            eurocEstGroundTruth gtData(time,pos,q,vel,bias_w,bias_a);
            eurocGroundTruthVec.push_back(gtData);
            Vector3d euler = Utility::Quaternion2ZYXEulerAngles(q);
            attRecord << gtData.time_s << "," << q.w() << "," << q.x() << "," << q.y() << ","
                      << q.z() << "," << euler.x() << "," << euler.y() << "," << euler.z() << endl;
            if(eurocGroundTruthVec.size() % 200 == 0 )
                printf("[ReadEurocData]:Euroc ground-truth current time_s is %f and recive size is %d!\n",gtData.time_s,eurocGroundTruthVec.size());
        }
    }
}

void readEurocData::readVisualCapturePosition(const string &_vcPos)
{
    ifstream vcPosFile(_vcPos.c_str());
    if(!vcPosFile)
    {
        printf("[ReadEurocData]:Path %s is not exsit!Please input right imu file path!\n",_vcPos.c_str());
        return;
    }
    string firstline;
    getline(vcPosFile,firstline);
    while(!vcPosFile.eof())
    {
        string lineStr[4];
        long int time;
        Vector3d pos;
        for(int i =0;i<4;i++)
        {
            if(i < 3)
            {
                getline(vcPosFile,lineStr[i],',');
            }else {
                getline(vcPosFile,lineStr[i]);
            }
            if(i == 0)
                time =atol(lineStr[i].c_str());
            else
                pos[i-1] = atof(lineStr[i].c_str());

        }
        if(!lineStr->empty())
        {
            eurocVisualCapturePos vcPosData(time,pos);
            eurocVCPosVec.push_back(vcPosData);
            if(eurocVCPosVec.size() % 100 == 0)
                printf("[ReadEurocData]:Euroc visual-capture pos current time_s is %f and recive size is %d!\n",vcPosData.time_s,eurocVCPosVec.size());

        }
    }
}
