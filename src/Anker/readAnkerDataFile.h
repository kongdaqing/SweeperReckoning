#ifndef READANKERDATAFILE_H
#define READANKERDATAFILE_H
#include "ankerDataType.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <math.h>

using namespace std;

class ReadAnkerDataFile
{
public:
    ReadAnkerDataFile(string dataPath);
    queue<AnkerData> AnkerDataSet;
private:
    bool type_is_int;
    ofstream recordData;
};

#endif // READANKERDATAFILE_H
