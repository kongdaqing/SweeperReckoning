#ifndef OPTFLOW_H
#define OPTFLOW_H
#include "sensorType.hpp"
#include "../Transform/Transform.hpp"
#include <fstream>
using namespace std;


class OptFlow{
public:
    OptFlow(string _configFile);
    ~OptFlow(){};
    Eigen::Vector3d TransformOpt2Odom(const OptFlowType &_opt,const IMUType& _imu); //tranform optflow data from opt frame to odomframe
    void DeadReckoningUpdate(const OptFlowType &_opt,const IMUType& _imu);
    const bool GetOptflowEstimatorState(){return  goodStateFlg;};
private:
double freq;
Eigen::Matrix<double,3,3> T;
OptFlowType lastData;
Transform2D T_opt;
bool goodStateFlg;
ofstream recordFile;
};




#endif
