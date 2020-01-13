#include "mahonyAttEstimator.hpp"

MahonyAttEstimator::MahonyAttEstimator(string _ConfigFile)
{
    imu = new IMU(_ConfigFile);
    cv::FileStorage fsSetting(_ConfigFile,cv::FileStorage::READ);
    string recordPath = fsSetting["save_path"];
    recordPath = recordPath + "attitude.csv";
    recordFile.open(recordPath);
    recordFile << "t" << "," << "q_w" << "," << "q_x" << "," << "q_y" << "," << "qz" << "," << "roll(rad)" << ","
               << "pitch(rad)" << "," << "yaw(rad)" << "," << "acc_roll(rad)" << "," << "acc_pitch(rad)" << ","  << "acc_yaw(rad)" <<  endl;
    recordFile << std::fixed;
    recordFile.precision(6);
    initialSuccessFlg = false;
    initalEuler.setZero();
    q.setIdentity();
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

void MahonyAttEstimator::SetAttitude(Eigen::Vector3d &_euler)
{
    euler = _euler;
    Utility::Euler2Quadnion(euler,q);
}
void MahonyAttEstimator::SetInitialAttitude(Eigen::Vector3d &_euler)
{
    initalEuler = _euler;
}
void MahonyAttEstimator::SetQuadnion(Eigen::Quaterniond &_quad)
{
    q = _quad;
    euler = Utility::Quaternion2EulerAngles(q);
}

void MahonyAttEstimator::InitializeEstimator(IMUType&_RawIMU)
{
    static Eigen::Vector3d sumAcc{Eigen::Vector3d::Zero()};
    static Eigen::Vector3d sumGyro{Eigen::Vector3d::Zero()};
    static int countSum = 0;
    if(initialSuccessFlg)
        return;
    Eigen::Vector3d resAcc;
    Eigen::Vector3d resGyr;
    resAcc = imu->GetCalibrAccData(_RawIMU.acc);
    resGyr = imu->GetCalibrGyroData(_RawIMU.gyro);
    if(resGyr.norm()*RAD2DEG > STEADY_GYRO_MINIRATE)
    {
        printf("[Estimator]:Please keep robot steay for Initialization!!!\n");
        sumAcc.setZero();
        sumGyro.setZero();
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
        initialSuccessFlg = InitializeAttitude(averAcc);
        if(initialSuccessFlg)
        {
            last_time = _RawIMU.time_s;
            lastAcc = resAcc;
            lastGyro = resGyr;
            accLPF.apply(resAcc,-1);
            gyrLPF.apply(resGyr,-1);
        }
        sumAcc.setZero();
        sumGyro.setZero();
        countSum = 0;
    }
}

bool MahonyAttEstimator::InitializeAttitude(Eigen::Vector3d &_Acc)
{
    Eigen::Vector3d tiltEuler = Utility::Acc2TiltAngle(_Acc);
    initalEuler[0] = tiltEuler[0];
    initalEuler[1] = tiltEuler[1];
/*
    double tiltAngleNorm = sqrt(initalEuler[0]*initalEuler[0] + initalEuler[1]*initalEuler[1]);
    if(tiltAngleNorm*RAD2DEG > 20){
        printf("[Estimator]: Initailization Failure because robot is not flat on the ground,you should put it flat!\n");
        return false;
    } else {
        Utility::Euler2Quadnion(initalEuler,q);
        euler = Utility::Quaternion2EulerAngles(q);
        printf("[Estimator]: Intialization Success!Theta = %f, Phi = %f \n",initalEuler[0]*RAD2DEG,initalEuler[1]*RAD2DEG);
    }*/
    Utility::Euler2Quadnion(initalEuler,q);
    euler = Utility::Quaternion2EulerAngles(q);
    printf("[Estimator]: Intialization Success!Theta = %f, Phi = %f \n",initalEuler[0]*RAD2DEG,initalEuler[1]*RAD2DEG);
    return  true;
}

void MahonyAttEstimator::EstimateAttitude(IMUType&_RawIMU)
{
    Eigen::Vector3d resAcc = imu->GetCalibrAccData(_RawIMU.acc);
    Eigen::Vector3d resGyro = imu->GetCalibrGyroData(_RawIMU.gyro);
    double time_s = _RawIMU.time_s;
    if(last_time < 0)
    {
        last_time = time_s;
        lastAcc = resAcc;
        lastGyro = resGyro;
        return;
    }
    double delta_time_s = time_s - last_time;
    last_time = time_s;
    double halfT = 0.5*delta_time_s;
    Eigen::Vector3d lpfAcc,lpfGyro;
    lpfAcc = accLPF.apply(resAcc,delta_time_s);
    lpfGyro = gyrLPF.apply(resGyro,delta_time_s);
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
    Eigen::Vector3d midAcc = 0.5*(lastAcc + lpfAcc);
    Eigen::Vector3d midGyr = 0.5*(lastGyro + lpfGyro);
    lastAcc = lpfAcc;
    lastGyro = lpfGyro;
    //归一化操作必不可少
    midAcc /= midAcc.norm();
    //Mahony attitude estimate
    Eigen::Vector3d G_w{Eigen::Vector3d(0,0,1)};
    //KDQ:一定要注意这里是q的逆才表示的是从世界坐标系到b系
    Eigen::Vector3d G_b = q.inverse()*G_w;
    Eigen::Vector3d err = Utility::Vec2SkewSymmeticMatrix(midAcc) * G_b;
    errInt += KI*halfT*err;
    Eigen::Vector3d fixGyro = errInt + KP*err;
    midGyr += fixGyro;
    cout << "[Estimator]: G_b is " << G_b.transpose() << "  Acc is " << midAcc.transpose() << " Fix gyro is " << fixGyro.transpose() << endl ;
    Eigen::Vector3d halfGyro = halfT*midGyr;
    Eigen::Quaterniond dq(1,halfGyro[0],halfGyro[1],halfGyro[2]);
    q = q*dq;
    q.normalize();
    euler = Utility::Quaternion2EulerAngles(q);
    Eigen::Vector3d acc2euler = Utility::Acc2TiltAngle(_RawIMU.acc);

    double  time_left_s = fmod(time_s,10000.0);
    recordFile << time_s << "," << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "," << euler[0] << ","
               << euler[1] << "," << euler[2] << "," << acc2euler[0] << "," << acc2euler[1] << "," << acc2euler[2] << endl;
    cout << "[Estimator]:"<<"Timestamp " << time_left_s << " Est quaternion is " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
    printf("[Estimator]:Current attitude angle roll-pitch-yaw : %f, %f, %f; Acc2Euler is %f %f\n",euler[0]*RAD2DEG,euler[1]*RAD2DEG,euler[2]*RAD2DEG,acc2euler[0]*RAD2DEG,acc2euler[1]*RAD2DEG);
}























