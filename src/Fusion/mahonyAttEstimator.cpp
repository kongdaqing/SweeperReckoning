#include "mahonyAttEstimator.hpp"

MahonyAttEstimator::MahonyAttEstimator(string _ConfigFile)
{
    imu = new IMU(_ConfigFile);
    cv::FileStorage fsSetting(_ConfigFile,cv::FileStorage::READ);
    string recordPath = fsSetting["save_att"];
    recordFile.open(recordPath);
    initialSuccessFlg = false;
    initalEuler.setZero();
    q.setIdentity();
    R.setIdentity();
    euler.setZero();
    errInt.setZero();
    last_time = -1;
    accLPF.set_cutoff_frequency(imu->freq,FREQUENCY_CUTOFF);
    gyrLPF.set_cutoff_frequency(imu->freq,FREQUENCY_CUTOFF);
    printf("[Estimator]:Record file is %s\n",recordPath.c_str());
    printf("[Estimator]:LPF cutoff frequency is %f\n",FREQUENCY_CUTOFF);
}
MahonyAttEstimator::~MahonyAttEstimator()
{
    recordFile.close();
    delete imu;
}

void MahonyAttEstimator::InitializeEstimator(Eigen::Vector3d &_RawAcc, Eigen::Vector3d &_RawGyro)
{
    static Eigen::Vector3d sumAcc{Eigen::Vector3d::Zero()};
    static Eigen::Vector3d sumGyro{Eigen::Vector3d::Zero()};
    static int countSum = 0;
    if(initialSuccessFlg)
        return;
    Eigen::Vector3d resAcc;
    Eigen::Vector3d resGyr;
    resAcc = imu->GetCalibrAccData(_RawAcc);
    resGyr = imu->GetCalibrGyroData(_RawGyro);
    if(resGyr.norm()*RAD2DEG > STEADY_GYRO_MINIRATE)
    {
        printf("[Estimator]:Please keep robot steay for Initialization!!!\n");
        sumAcc.setZero();
        countSum = 0;
        return;
    }
    countSum++;
    sumAcc += resAcc;
    sumGyro += resGyr;
    if(countSum == INITIALCOUNT)
    {
        Eigen::Vector3d averAcc = sumAcc/INITIALCOUNT;
        Eigen::Vector3d averGyro = sumGyro/INITIALCOUNT;
        imu->SetCalibrRandomGyroBias(averGyro);
        InitializeAttitude(averAcc);
        sumAcc.setZero();
        sumGyro.setZero();
        countSum = 0;
    }
}

void MahonyAttEstimator::InitializeAttitude(Eigen::Vector3d &_Acc)
{
    Eigen::Vector3d normlizedAcc = _Acc/_Acc.norm();
    initalEuler[0] = atan2(normlizedAcc.y(),normlizedAcc.z());
    initalEuler[1] = asin(-normlizedAcc.x());
    double tiltAngleNorm = sqrt(initalEuler[0]*initalEuler[0] + initalEuler[1]*initalEuler[1]);
   /*
    if(tiltAngleNorm*RAD2DEG > 10)
        printf("[Estimator]: Initailization Failure because robot is not flat on the ground,you should put it flat!\n");
    else {*/
       initialSuccessFlg = true;
       Utility::Euler2Quadnion(initalEuler,q);
       R = q.toRotationMatrix();
       euler = Utility::Quaternion2EulerAngles(q);
       printf("[Estimator]: Intialization Success!Theta = %f, Phi = %f \n",initalEuler[0]*RAD2DEG,initalEuler[1]*RAD2DEG);
       printf("[Estimator]: Intialization Success!Theta = %f, Phi = %f \n",euler[0]*RAD2DEG,euler[1]*RAD2DEG);
   // }
}

void MahonyAttEstimator::EstimateAttitude(Eigen::Vector3d &_RawAcc, Eigen::Vector3d &_RawGyro,double time_s)
{
    Eigen::Vector3d resAcc = imu->GetCalibrAccData(_RawAcc);
    Eigen::Vector3d resGyro = imu->GetCalibrGyroData(_RawGyro);

    if(last_time < 0)
    {
        last_time = time_s;
        return;
    }
    double delta_time_s = time_s - last_time;
    last_time = time_s;

    if(delta_time_s <= 0)
    {
        printf("[Estimator]:Timestamp sequence is not right,please check timestamp!!!\n");
        estimatorState.sequenceUnusualCount++;
        if(estimatorState.sequenceUnusualCount > 100)
        {
            estimatorState.SuccessFlg = false;
            errInt.setZero();
            printf("[Estimator]:Timestamp sequence is not right,Estimator is corrupt!\n");
        }
        return;
    }else if (delta_time_s > 1.0) {
        estimatorState.intervalUnusualCount++;
        printf("[Estimator]:Timestamp interval is too big,please check timestamp!!!\n");
        if(estimatorState.intervalUnusualCount > 10)
        {
            estimatorState.SuccessFlg = false;
            errInt.setZero();
            printf("[Estimator]:Timestamp interval is too big,Estimator is corrupt!\n");
        }
        return;
    }
    Eigen::Vector3d lpfAcc,lpfGyro;
    lpfAcc = accLPF.apply(resAcc,delta_time_s);
    printf("[Estimator]:Acc(%f,%f,%f) ACCLPF(%f,%f,%f)\n",resAcc[0],resAcc[1],resAcc[2],lpfAcc[0],lpfAcc[1],lpfAcc[2]);
    lpfGyro = gyrLPF.apply(resGyro,delta_time_s);
    //Mahony attitude estimate
    double halfT = 0.5*delta_time_s;
    lpfAcc /= lpfAcc.norm();
    Eigen::Vector3d G_w{Eigen::Vector3d(0,0,1)};
    Eigen::Vector3d G_b = R*G_w;
    Eigen::Vector3d err = Utility::Vec2SkewSymmeticMatrix(lpfAcc) * G_b;
    errInt += KI*halfT*err;
    lpfGyro += KP*err + errInt;
    cout << "[Estimator]: G_b is " << G_b.transpose() << "   Acc is " << lpfAcc.transpose() << endl;
    Eigen::Vector3d halfGyro = halfT*lpfGyro;
    Eigen::Quaterniond dq(1,halfGyro[0],halfGyro[1],halfGyro[2]);
    q = q*dq;
    q.normalize();
    R = q.toRotationMatrix();
    euler = Utility::Quaternion2EulerAngles(q);
    printf("[Estimator]:Current attitude angle roll-pitch-yaw : %f, %f, %f \n",euler[0]*RAD2DEG,euler[1]*RAD2DEG,euler[2]*RAD2DEG);
    recordFile << time_s << "," << euler[0]*RAD2DEG << "," << euler[1]*RAD2DEG << "," << euler[2]*RAD2DEG << endl;

}























