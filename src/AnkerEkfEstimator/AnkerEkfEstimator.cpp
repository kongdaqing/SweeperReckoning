#include "AnkerEkfEstimator.h"

AnkerEkfEstimator::AnkerEkfEstimator(ekf_state _state,ekf_motionNoise _motion_noise,ekf_measurementNoise _measure_noise)
  :state(_state),motion_noise(_motion_noise),measue_noise(_measure_noise)
{
  matrix.F = Eigen::MatrixXd::Identity(6,6);
  matrix.F(4,4) = 0;
  matrix.F(4,5) = -1.0f;

  matrix.Q = Eigen::MatrixXd::Zero(3,3);
  matrix.Q(0,0) = motion_noise.body_vel_noise*motion_noise.body_vel_noise;
  matrix.Q(1,1) = motion_noise.gyro_noise*motion_noise.gyro_noise;
  matrix.Q(2,2) = motion_noise.gyro_bias_noise*motion_noise.gyro_bias_noise;
  matrix.G = Eigen::MatrixXd::Zero(6,3);
  matrix.G(3,0) = 1.0f;
  matrix.G(4,1) = 1.0f;
  matrix.G(5,2) = 1.0f;


  matrix.H = Eigen::MatrixXd::Zero(2,6);
  matrix.H(0,3) =  1.0f;
  matrix.H(0,4) = -0.5f*wheel_distance;
  matrix.H(1,3) =  1.0f;
  matrix.H(1,4) =  0.5f*wheel_distance;
  matrix.R = Eigen::MatrixXd::Zero(2,2);
  matrix.R(0,0) = measue_noise.odometry_vel_l_noise_q*measue_noise.odometry_vel_l_noise_q;
  matrix.R(1,1) = measue_noise.odometry_vel_r_noise_q*measue_noise.odometry_vel_r_noise_q;
  matrix.M = Eigen::MatrixXd::Identity(2,2);

  matrix.P = 0.01f*Eigen::MatrixXd::Identity(6,6);
//  std::cout << "P: \n" << matrix.P << std::endl;
//  std::cout << "F: \n" << matrix.F << std::endl;
//  std::cout << "Q: \n" << matrix.Q << std::endl;
//  std::cout << "R: \n" << matrix.R << std::endl;
//  std::cout << "G: \n" << matrix.G << std::endl;
//  std::cout << "M: \n" << matrix.M << std::endl;
//  std::cout << "H: \n" << matrix.H << std::endl;
}
Eigen::Matrix<double,6,1> AnkerEkfEstimator::ankerEkfFusion(AnkerDataType& cur_data,double delta_time_s)
{
  //Step1: predict state
  Eigen::Matrix<double,6,1> predict_state;
  predict_state(0,0) = state.w_px + state.b_v*cos(state.yaw)*delta_time_s;
  predict_state(1,0) = state.w_py + state.b_v*sin(state.yaw)*delta_time_s;
  predict_state(2,0) = state.yaw  + state.w*delta_time_s;
  predict_state(3,0) = state.b_v;
  predict_state(4,0) = cur_data.Gyro[2] - state.w_bias;
  predict_state(5,0) = state.w_bias;
  //predict covariance
  updateJacbianF(delta_time_s);
  Eigen::Matrix<double,6,6>  predict_P = predictCovariance();
  //Step2: Calculate gain matrix
  Eigen::Matrix<double,6,2>  gain_matrix = calculateGainMatrix(predict_P);

  //step3: fix state
  Eigen::Matrix<double,2,1> predict_measure,measurement;
  predict_measure(0,0) = predict_state(3,0) - 0.5f*wheel_distance*predict_state(4,0);
  predict_measure(1,0) = predict_state(3,0) + 0.5f*wheel_distance*predict_state(4,0);
  measurement(0,0) = cur_data.Odometry_vel[0];
  measurement(1,0) = cur_data.Odometry_vel[1];
  Eigen::Matrix<double,6,1> fix_state;
  fix_state = predict_state + gain_matrix*(measurement - predict_measure);
  updateCovariance(gain_matrix,predict_P);
  updateState(fix_state);
  return fix_state;
}

void AnkerEkfEstimator::updateJacbianF(double delta_time_s)
{
  matrix.F(0,2) = -sin(state.yaw)*state.b_v*delta_time_s;
  matrix.F(0,3) =  cos(state.yaw)*delta_time_s;
  matrix.F(1,2) =  cos(state.yaw)*state.b_v*delta_time_s;
  matrix.F(1,3) =  sin(state.yaw)*delta_time_s;
  matrix.F(2,4) =  delta_time_s;
}
void AnkerEkfEstimator::updateCovariance(Eigen::Matrix<double,6,2>& gain_matrix,Eigen::Matrix<double,6,6>& predict_P)
{
  matrix.P = (Eigen::MatrixXd::Identity(6,6) - gain_matrix*matrix.H)*predict_P;
}
Eigen::Matrix<double,6,6> AnkerEkfEstimator::predictCovariance()
{
  Eigen::Matrix<double,6,6> predict_P = matrix.F*matrix.P*matrix.F.transpose() + matrix.G*matrix.Q*matrix.G.transpose();
  return predict_P;

}
Eigen::Matrix<double,6,2> AnkerEkfEstimator::calculateGainMatrix(Eigen::Matrix<double,6,6>& predict_P)
{
  Eigen::Matrix<double,2,2>  tmp_matrix = matrix.H*predict_P*matrix.H.transpose() + matrix.M*matrix.R*matrix.M.transpose();
  Eigen::Matrix<double,6,2>  gain_matrix = predict_P*matrix.H.transpose()*tmp_matrix.inverse();
  return gain_matrix;
}

void AnkerEkfEstimator::updateState(Eigen::Matrix<double,6,1>& new_state)
{

  state.w_px = new_state(0,0);
  state.w_py = new_state(1,0);
  state.yaw = new_state(2,0);
  state.b_v = new_state(3,0);
  state.w =   new_state(4,0);
  state.w_bias = new_state(5,0);
}















