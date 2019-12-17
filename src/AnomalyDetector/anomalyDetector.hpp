#ifndef ANOMALYDETECTOR_H
#define ANOMALYDETECTOR_H

#include "../Sensor/sensorType.hpp"
#include <fstream>
using namespace std;

class AnomalyDetector{
public:
AnomalyDetector(string _configFile);
~AnomalyDetector();
bool DetectOdomOptState(const IMUType& _imu,const OptFlowType& _opt,const OdometryType& _odom);


private:
ofstream recordFile;

};

#endif
