#include "readAnkerDataFile.h"

ReadAnkerDataFile::ReadAnkerDataFile(const string& imu_file,const string& odometry_file,const string& optical_file)
{

  ifstream imu_fin(imu_file.c_str());
  if(!imu_fin)
  {
    printf("File %s doesn't exist!",imu_file.c_str());
    return;
  }
  ifstream odo_fin(odometry_file.c_str());
  if(!odo_fin)
  {
    printf("File %s doesn't exist!",odometry_file.c_str());
    return;
  }
  ifstream opt_fin(optical_file.c_str());
  if(!opt_fin)
  {
    printf("File %s doesn't exist!",optical_file.c_str());
    return;
  }
  printf("NOW!Load anker sensor from files!");
  while(!imu_fin.eof())
  {
    double t_imu,t_odo,t_opt;
    imu_fin >> t_imu;
    odo_fin >> t_odo;
    opt_fin >> t_opt;
    long int time_ns = t_imu;
    t_imu *= TIME_Resolution;
    t_odo *= TIME_Resolution;
    t_opt *= TIME_Resolution;
    if(fabs(t_imu - t_odo) > 0.004f || fabs(t_imu - t_opt) > 0.004f)
    {
      printf("IMU and ODO time stamp is not sync!We shut down system!");
      return;
    }
    AnkerData raw_data;
    raw_data.time = t_imu;
    raw_data.time_ns = time_ns;
    imu_fin >> raw_data.ax;
    imu_fin >> raw_data.ay;
    imu_fin >> raw_data.az;
    imu_fin >> raw_data.gx;
    imu_fin >> raw_data.gy;
    imu_fin >> raw_data.gz;

    odo_fin >> raw_data.odo_right_pos;
    odo_fin >> raw_data.odo_left_pos;
    odo_fin >> raw_data.odo_right_vel;
    odo_fin >> raw_data.odo_left_vel;

    opt_fin >> raw_data.opt_pos_x;
    opt_fin >> raw_data.opt_pos_y;
    opt_fin >> raw_data.wall_distance_right;
    opt_fin >> raw_data.wall_distance_left;
    opt_fin >> raw_data.opt_quality;


    raw_data.ax *= ACC_Resolution;
    raw_data.ay *= ACC_Resolution;
    raw_data.az *= ACC_Resolution;
    raw_data.gx *= GYRO_RadResolution;
    raw_data.gy *= GYRO_RadResolution;
    raw_data.gz *= GYRO_RadResolution;

    raw_data.odo_right_pos *= ODO_Resolution;
    raw_data.odo_left_pos *= ODO_Resolution;
    raw_data.odo_right_vel *= ODO_Resolution;
    raw_data.odo_left_vel *= ODO_Resolution;
    raw_data.opt_pos_x *= OPT_Resolution;
    raw_data.opt_pos_y *= OPT_Resolution;

    AnkerDataSet.push(raw_data);
  }
  printf("Finish loading!");
  printf("Load anker sensor datas size: %ld",AnkerDataSet.size());
}
