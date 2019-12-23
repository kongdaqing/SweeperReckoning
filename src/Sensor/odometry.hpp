#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../Transform/Transform.hpp"
#include "sensorType.hpp"

using namespace std;




class Odometry
{
public:
    Odometry(string _configFile);
    ~Odometry();
    void GetOdometryVel(const OdometryType& _odoData,OdometryOptflowType& odomOptData);
    Eigen::Vector3d GetOdometryDeltaPose(const OdometryType& _odoData,const OdometryType &_lastOdom);

private:
double baseLine;
double freq;


};


#endif
