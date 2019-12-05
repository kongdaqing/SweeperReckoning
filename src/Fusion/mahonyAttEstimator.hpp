#ifndef MAHONYATTESTIMATOR_H
#define MAHONYATTESTIMATOR_H

#include "../Sensor/imu.hpp"
#include "../Utility/lowPassFilter.hpp"
#include "../Utility/utility.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#define STEADY_GYRO_MINIRATE 10
#define RAD2DEG 57.29578
#define INITIALCOUNT 100
#define FREQUENCY_CUTOFF 15.0
#define KP 10
#define KI 0.01


using namespace std;



class MahonyAttEstimator
{
public:
    MahonyAttEstimator(string _ConfigFile);
    ~MahonyAttEstimator();
    void InitializeEstimator(IMUType&_RawIMU);
    bool InitializeAttitude(Eigen::Vector3d& _Acc);
    void EstimateAttitude(IMUType&_RawIMU);
    void SetAttitude(Eigen::Vector3d& _euler);
    void SetInitialAttitude(Eigen::Vector3d& _euler);
    void SetQuadnion(Eigen::Quaterniond &_quad);
    struct EstState{
        EstState()
        {
            sequenceUnusualCount = 0;
            intervalUnusualCount = 0;
            SuccessFlg = true;
        }
        int sequenceUnusualCount;
        int intervalUnusualCount;
        bool SuccessFlg;
    };
    bool GetEstimatorState(){return estimatorState.SuccessFlg;};
    bool initialSuccessFlg;
    const Eigen::Quaterniond GetAttQuadnion(){return  q;};
private:
IMU *imu;
Eigen::Vector3d initalEuler;
Eigen::Quaterniond q;
Eigen::Vector3d euler;
double  last_time;
EstState estimatorState;
LowPassFilterVector3d accLPF;
LowPassFilterVector3d gyrLPF;
Eigen::Vector3d errInt;
ofstream recordFile;
Eigen::Vector3d lastAcc;
Eigen::Vector3d lastGyro;
};



#endif
