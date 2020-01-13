#ifndef EUROCDATATYPE_H
#define EUROCDATATYPE_H
#include <Eigen/Eigen>
using namespace Eigen;
struct eurocImuType
{
    long int timestamp;
    double time_s;
    Vector3d gyro;
    Vector3d acc;
}



#endif // EUROCDATATYPE_H
