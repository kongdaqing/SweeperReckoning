#ifndef ANKERESTIMATOR_H
#define ANKERESTIMATOR_H
#include "imu.h"
#include "AnkerEkfEstimator.h"
#include "ankerinfomation.h"

class AnkerEstimator
{
public:
  AnkerEstimator();
  bool EstimatorInitialization(AnkerDataType& data);
  AnkerPose EstimateAnkerPose(AnkerDataType& data);
  void ConstructAnkerEkfEstimator(AnkerDataType& data);
  IMU imu;
protected:
  AnkerEkfEstimator *ekf_estimator;

  double last_time;
};

#endif // ANKERESTIMATOR_H
