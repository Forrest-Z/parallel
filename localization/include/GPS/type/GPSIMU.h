#pragma once

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h> 
#include <stdlib.h>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <string>

using namespace std;

#define BUF_SIZE 1024
#define BYTE unsigned char
#define INFORMATION_FLAG ","
#define INFORMATION_GPFPD_HEADING_FLAG "PD"
#define INFORMATION_GTIMU_HEADING_FLAG "$GTIMU"
#define INFORMATION_ENDING_FLAG "*"

#define GPFPD_STRUCT_SIZE 14
#define GPFPD_STRUCT_PART_SIZE 9
#define GTIMU_STRUCT_SIZE 8


struct GPFPD 
{
    double GPSTime;
    double GPSWeek;
    double Heading;
    double Pitch;
    double Roll;
    double Latitude;
    double Longitude;
    double Altitude;
    double Ve;
    double Vn;
    double Vu;
    double Baseline;
    double NSV1;
    double NSV2;
    double Status;
    double Ax, Ay, Az, Wx, Wy, Wz;
};


struct GTIMU 
{
    double GPSTime;
    double GPSWeek;
    double GyroX;
    double GyroY;
    double GyroZ;
    double AccX;
    double AccY;
    double AccZ;
    double Tpr;
};

class GPSIMU
{
private:
    int code;
    string resultStr;
    vector<string> originStr;
    vector<double> gpfpdDataV;
    vector<double> gtimuDataV;
    vector<double> GGADataV;
    vector<double> HDTDataV;
    struct GPFPD gpfpdData;
    struct GTIMU gtimuData;

public:
    void parse(string msg);

    void GPFPDParser();
    void GTIMUParser();
    void GGAHDTParser();

    void setGPFPD();
    void setGTIMU();

    //double getGPSTime();
    //double getGPSWeek();
    double getHeading();
    double getRoll();
    double getPitch();
    double getLatitude();
    double getLongitude();

    double getAccX();
    double getAccY();
    double getAccZ();

    struct GPFPD getGPFPDStruct();
    struct GTIMU getGTIMUStruct();
};