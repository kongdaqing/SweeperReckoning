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
    void InitializeDeadReckoning(const OdometryType& _odoData);
    void DeadReckoningUpdate(const OdometryType& _odoData,const IMUType& _imuData);
    void SetTransform(const Transform2D& _T);
    const Transform2D GetTransform2D();
    void GetOdometryVel(const OdometryType& _odoData,OdometryOptflowType& odomOptData);
    Eigen::Vector3d GetOdometryDeltaPose(const OdometryType& _odoData,const OdometryType &_lastOdom);
    bool GetDeadReckoningState(){return  goodStateFlg;}
private:
double baseLine;
double freq;
Transform2D T,T_imu;
double lastTime_s;
bool initialFlg;
unsigned int badTimestampCount;
bool goodStateFlg;
ofstream recordFile;
OdometryType lastOdom;
};


#endif
