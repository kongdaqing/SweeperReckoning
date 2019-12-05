#pragma once
#include <Eigen/Dense>
#define pi 3.1415926
class Utility
{
public:

    static double ConstriantDouble(double x,double min,double max)
    {
        if(x > max)
            x = max;
        if(x < min)
            x = min;
        return x;
    }


    static void Euler2Quadnion(Eigen::Vector3d& euler,Eigen::Quaterniond& q)
    {
        Eigen::Vector3d halfEuler = euler/2;
        double sinHalfPhi = sin(halfEuler.x());
        double cosHalfPhi = cos(halfEuler.x());
        double sinHalfThe = sin(halfEuler.y());
        double cosHalfThe = cos(halfEuler.y());
        double sinHalfPsi = sin(halfEuler.z());
        double cosHalfPsi = cos(halfEuler.z());

        double qw = cosHalfPhi*cosHalfThe*cosHalfPsi + sinHalfPhi*sinHalfThe*sinHalfPsi;
        double qx = sinHalfPhi*cosHalfThe*cosHalfPsi - cosHalfPhi*sinHalfThe*sinHalfPsi;
        double qy = cosHalfPhi*sinHalfThe*cosHalfPsi + sinHalfPhi*cosHalfThe*sinHalfPsi;
        double qz = cosHalfPhi*cosHalfThe*sinHalfPsi - sinHalfPhi*sinHalfThe*cosHalfPsi;
        q = Eigen::Quaterniond(qw,qx,qy,qz);
        q.normalize();
    }
    static Eigen::Matrix3d Vec2SkewSymmeticMatrix(Eigen::Vector3d& vec)
    {
        Eigen::Matrix3d SkewMatrix;
        SkewMatrix <<     0,  -vec[2],  vec[1],
                     vec[2],        0, -vec[0],
                    -vec[1],   vec[0],       0;
        return  SkewMatrix;
    }

    static Eigen::Vector3d Quaternion2EulerAngles(Eigen::Quaterniond& q)
    {
        double q0 = q.w();
        double q1 = q.x();
        double q2 = q.y();
        double q3 = q.z();
        double roll,pitch,yaw;
        roll = atan2(2 * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);
        pitch = asin(2 * (q0*q2 - q1*q3));
        yaw = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
        if(yaw > pi)
            yaw = yaw - 2*pi;
        if(yaw < -pi)
            yaw = yaw + 2*pi;
        Eigen::Vector3d euler(roll,pitch,yaw);
        return euler;
    }
    static Eigen::Vector3d Acc2TiltAngle(Eigen::Vector3d& acc)
    {
        Eigen::Vector3d Euler;
        Eigen::Vector3d normlizedAcc = acc/acc.norm();
        
        Euler.setZero();
        Euler[0] = atan2(normlizedAcc.y(),normlizedAcc.z());
        Euler[1] = asin(-normlizedAcc.x());
        return Euler;
    }


};








