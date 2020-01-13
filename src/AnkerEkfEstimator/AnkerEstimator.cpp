#include "AnkerEstimator.h"

AnkerEstimator::AnkerEstimator():ekf_estimator(nullptr)
{

}
bool AnkerEstimator::EstimatorInitialization(AnkerDataType &data)
{

  bool flg = imu.calibrationGyroBias(data);
  if(flg == true)
  {
    ConstructAnkerEkfEstimator(data);
  }
  return flg;
}
void AnkerEstimator::ConstructAnkerEkfEstimator(AnkerDataType& data)
{
  ekf_state _state;
  _state.w_px = 0;
  _state.w_py = 0;
  _state.yaw = 0;
  _state.b_v = 0;
  _state.w = 0;
  _state.w_bias = imu.getGyroBias()[2];
  ekf_motionNoise _motion_noise;
  _motion_noise.body_vel_noise = 0.04f;
  _motion_noise.gyro_noise = gyro_z_noise_density;
  _motion_noise.gyro_bias_noise = gyro_z_noise_random_walk;
  ekf_measurementNoise _measurement_noise;
  _measurement_noise.odometry_vel_l_noise_q = 0.001f;
  _measurement_noise.odometry_vel_r_noise_q = 0.001f;
  ekf_estimator = new AnkerEkfEstimator(_state,_motion_noise,_measurement_noise);
  last_time =  data.time_s;
  std::printf("Construct Anker Ekf Estimator Finish!\n");
}


AnkerPose AnkerEstimator::EstimateAnkerPose(AnkerDataType &data)
{
  AnkerPose anker_pose;
   if(ekf_estimator == nullptr)
     return anker_pose;
   double delta_time = data.time_s - last_time;
   if(delta_time < 0.0f || delta_time > 0.2f)
   {
     delete ekf_estimator;
     std::printf("Time stamp is not right,Reset Ekf Estimator!\n");
     ConstructAnkerEkfEstimator(data);
     return anker_pose;
   }
   last_time = data.time_s;
   Eigen::Matrix<double,6,1> ekf_state = ekf_estimator->ankerEkfFusion(data,delta_time);
   anker_pose.position[0] = ekf_state[0];
   anker_pose.position[1] = ekf_state[1];
   anker_pose.euler_rad[2] = ekf_state[2];
   anker_pose.euler2quaternion();
   std::printf("anker robot pose: (%f,%f,%f)\n",anker_pose.position[0],anker_pose.position[1],anker_pose.euler_rad[2]*rad2deg);
   return anker_pose;
}


















