#ifndef OPTFLOW_H
#define OPTFLOW_H
#include "sensorType.hpp"
#include "../Transform/Transform.hpp"
#include "../Utility/utility.hpp"


using namespace std;


class OptFlow{
public:
    OptFlow(string _configFile);
    ~OptFlow(){};
    Eigen::Vector3d TransformOpt2Odom(const OptFlowType &_opt,const OptFlowType & _lastOpt,const IMUType& _imu); //tranform optflow data from opt frame to odomframe
    void GetOdoVelFromOpt(const OptFlowType &_opt,const IMUType& _imu,OdometryOptflowType& odomOptData);
    void DeadReckoningUpdate(const OptFlowType &_opt,const IMUType& _imu);
    const bool GetOptflowEstimatorState(){return  goodStateFlg;};
    void CheckOptflowDataQuality(const OptFlowType& _opt);
    bool GetOptflowDataQualityFlg(){return optflowGoodQualityFlg;};
private:
double freq;
double optIqThrMax;
double optIqTimeMin;
bool goodStateFlg;
bool optflowGoodQualityFlg;
Eigen::Matrix<double,3,3> T;
ofstream recordFile;
Transform2D T_opt;
};




#endif
