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
    void GetOdoVelFromOpt(const OptFlowType &_opt,const OptFlowType & _lastOpt,const IMUType& _imu,OdometryOptflowType& odomOptData);
    void CheckOptflowDataQuality(const OptFlowType& _opt);
    bool GetOptflowDataQualityFlg(){return optflowGoodQualityFlg;};
private:
double freq;
double optIqThrMax;
double optIqTimeMin;
bool optflowGoodQualityFlg;
Eigen::Matrix<double,3,3> T;

};




#endif
