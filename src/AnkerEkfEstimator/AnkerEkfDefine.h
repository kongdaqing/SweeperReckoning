#ifndef ANKEREKFDEFINE_H
#define ANKEREKFDEFINE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
struct ekf_state
{
    double w_px;
    double w_py;
    double yaw;
    double b_v;
    double w;
    double w_bias;
};
struct ekf_matrix{

    Eigen::Matrix<double,6,6> P;
    Eigen::Matrix<double,6,6> F;
    Eigen::Matrix<double,3,3> Q;
    Eigen::Matrix<double,6,3> G;
    Eigen::Matrix<double,2,6> H;
    Eigen::Matrix<double,2,2> R;
    Eigen::Matrix<double,2,2> M;

};
struct ekf_motionNoise{
    double body_vel_noise;
    double gyro_noise;
    double gyro_bias_noise;
};
struct ekf_measurementNoise
{
    double odometry_vel_l_noise_q;
    double odometry_vel_r_noise_q;
};
#endif // ANKEREKFDEFINE_H
