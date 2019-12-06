#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../Transform/Transform.hpp"
#include "sensorType.hpp"
#include <fstream>
using namespace std;
#define RAD2DEG 57.29578




class Odometry
{
public:
    Odometry(string _configFile);
    ~Odometry();
    void InitializeDeadReckoning(OdometryType& _odoData);
    void DeadReckoningUpdate(OdometryType& _odoData,double _imuRate_z);
    void SetTransform(Transform2D& _T);
    const Transform2D GetTransform2D();
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
};


#endif
