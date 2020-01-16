#ifndef EUROCDATATYPE_H
#define EUROCDATATYPE_H
#include <Eigen/Eigen>
using namespace Eigen;
struct eurocImu
{
    eurocImu()
    {}
    eurocImu(long int _time,Vector3d& _gyro,Vector3d& _acc)
    {
       timestamp = _time;
       time_s = _time * 1e-09;
       gyro = _gyro;
       acc = _acc;
    }
    long int timestamp;
    double time_s;
    Vector3d gyro;
    Vector3d acc;
};

struct eurocVisualCapturePos
{
    eurocVisualCapturePos(){}
    eurocVisualCapturePos(long int _time,Vector3d& _pos)
    {
      timestamp = _time;
      time_s = _time * 1e-09;
      pos = _pos;
    }
    long int timestamp;
    double time_s;
    Vector3d pos;
};

struct eurocEstGroundTruth
{
    eurocEstGroundTruth(){}
    eurocEstGroundTruth(long int _time,Vector3d& _pos,Quaterniond& _q,Vector3d& _vel,
                        Vector3d& _bias_w,Vector3d& _bias_a)
    {
        timestamp = _time;
        time_s = _time * 1e-09;
        pos = _pos;
        q = _q;
        vel = _vel;
        bias_w = _bias_w;
        bias_a = _bias_a;
    }

    long int timestamp;
    double time_s;
    Vector3d pos;
    Quaterniond q;
    Vector3d vel;
    Vector3d bias_w;
    Vector3d bias_a;
};


#endif // EUROCDATATYPE_H
