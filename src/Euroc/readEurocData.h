#ifndef READEUROCDATA_H
#define READEUROCDATA_H
#include <iostream>
#include <fstream>
#include <vector>
#include "eurocDataType.h"
using namespace std;

class readEurocData
{
public:
    readEurocData(const string& _dataPath);
    void readIMUData(const string& _imuPath);
    void readEstGroundTruth(const string& _gtPath);
    void readVisualCapturePosition(const string& _vcPos);
    vector<eurocImu> eurocImuVec;
    vector<eurocEstGroundTruth> eurocGroundTruthVec;
    vector<eurocVisualCapturePos> eurocVCPosVec;
    ofstream attRecord;
};

#endif // READEUROCDATA_H
