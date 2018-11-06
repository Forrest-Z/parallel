#include <GPS/type/GPSIMU.h>
#include <iomanip>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void GPSIMU::parse(string msg)
{
    // 将连续的字符串切分为多个独立的字符串，并形成字符串向量。
    boost::split(originStr, msg, boost::is_any_of(INFORMATION_FLAG), boost::token_compress_on);
    GGAHDTParser();
}

void GPSIMU::GGAHDTParser()
{
    string firstStr=originStr[0];
    string tempStr="";

    int GGAHDTHeadingFlag=firstStr.find("GA");
    int HDTHeadingFlag=-1;
    int HDTNum=0;
    if(GGAHDTHeadingFlag!=-1)
    {
        std::cout<<"OK"<<endl;
        int i=-1;
        for(auto it=originStr.begin(); it!=originStr.end(); ++it)
        {
            tempStr=*it;
            i=i+1;
            HDTHeadingFlag=tempStr.find("DT");
            if(HDTHeadingFlag!=-1)
            {
                HDTNum=i;
                break;
            }
        }   

        if(originStr.size() < 5 || originStr.size() < HDTNum+2)
        {
            cout << "Wrong Data" << endl;
            return;
        }

        long LATIN= (long)(atof(originStr[2].c_str() )*1000000);
        long LONIN= (long)(atof( originStr[4].c_str())*100000);

        gpfpdData.GPSWeek =1.0;
        gpfpdData.GPSTime = 2.0;
        gpfpdData.Heading = atof(originStr[HDTNum+1].c_str() );
        gpfpdData.Pitch = 4.0;
        gpfpdData.Roll = 5.0;
        gpfpdData.Latitude =LATIN/100000000+(LATIN%100000000)/1000000.00/60;
        gpfpdData.Longitude = LONIN/10000000+(LONIN%10000000)/100000.00/60;
        gpfpdData.Altitude = 8.0;
        gpfpdData.Ve = 9.0;
        gpfpdData.Vn = 10.0;
        gpfpdData.Vu = 11.0;
        gpfpdData.Baseline =12.0;
        gpfpdData.NSV1 = 13.0;
        gpfpdData.NSV2 = 14.0;
        std::cout<<setprecision(10)
            << "gpfpdData.Latitude : "<<gpfpdData.Latitude<< endl 
            << "gpfpdData.Longitude: "<<gpfpdData.Longitude<<endl
            << "gpfpdData.Heading  : "<<gpfpdData.Heading<<endl
            << "GPSTime: " << gpfpdData.GPSTime << endl <<endl;
    }
    else
    {   
        std::cout<<"NOT FOUND"<<endl;
    }
}

void GPSIMU::GPFPDParser() 
{
    string firstStr = originStr[0];
    string tempStr = "";
    int GPFPDHeadingFlag = firstStr.find(INFORMATION_GPFPD_HEADING_FLAG);//检测起始每条GPS信息的标记位。
    int GPFPDEndingFlag = -1;
    gpfpdDataV.clear();

    if (GPFPDHeadingFlag != -1) //如果找到GPS起始标记位
    {
        for(vector<string>::iterator it = originStr.begin(); it != originStr.end(); ++it) 
        {
            tempStr = *it;
            GPFPDEndingFlag = tempStr.find(INFORMATION_ENDING_FLAG);//逐个寻找GPS的结束位
            if (GPFPDEndingFlag == -1) 
                //如果没有找到结束位，则将现有的数据转化成浮点数，存储在gpfpdDataV向量中。 
            {
                gpfpdDataV.push_back(atof(tempStr.c_str()));
            }
            else //如果找到了结束标志位，则将一条完整的GPS信息赋值给相应的结构体。 
            {
                setGPFPD();
                break;
            }
        }
    }
    else 
    {
  //      std::cout << "first part of origin str: " << firstStr << std::endl;
    }
}

void GPSIMU::GTIMUParser() 
{
    int GTIMUHeadingFlag = -1;
    int GTIMUEndingFlag = -1;
    int GPFPDHeadingFlag = -1; 
    bool GTIMUParserFlag = false;
    string tempStr = "";

    for(vector<string>::iterator it = originStr.begin(); it != originStr.end(); ++it) 
    {
        tempStr = *it;
        GTIMUHeadingFlag = tempStr.find(INFORMATION_GTIMU_HEADING_FLAG);
        GTIMUEndingFlag = tempStr.find(INFORMATION_ENDING_FLAG);
        GPFPDHeadingFlag = tempStr.find(INFORMATION_GPFPD_HEADING_FLAG);

        if (GTIMUParserFlag == true) 
        {
            if (GTIMUEndingFlag == -1) 
            {
                gtimuDataV.push_back(atof(tempStr.c_str()));
            }
            else 
            {
                setGTIMU();
                gtimuDataV.clear();
            }
        }

        if (GTIMUHeadingFlag != -1)
        {
             GTIMUParserFlag = true;
        }
        else if (GTIMUEndingFlag != -1) 
        {
            GTIMUParserFlag = false;
            gtimuDataV.clear();
        }
    }
}

double GPSIMU::getHeading() {
    return gpfpdData.Heading;
}


double GPSIMU::getRoll() {
    return gpfpdData.Roll;
}


double GPSIMU::getPitch() {
    return gpfpdData.Pitch;
}


double GPSIMU::getLatitude() {
    return gpfpdData.Latitude;
}


double GPSIMU::getLongitude() {
    return gpfpdData.Longitude;
}


double GPSIMU::getAccX() {
    return gtimuData.AccX;
}


double GPSIMU::getAccY() {
    return gtimuData.AccY;
}
    
double GPSIMU::getAccZ() {
    return gtimuData.AccZ;
}

void GPSIMU::setGPFPD() 
{
    if (gpfpdDataV.size() >= GPFPD_STRUCT_SIZE) 
    {
        gpfpdData.GPSWeek = gpfpdDataV[1];
        gpfpdData.GPSTime = gpfpdDataV[2];
        gpfpdData.Heading = gpfpdDataV[3];
        gpfpdData.Pitch = gpfpdDataV[4];
        gpfpdData.Roll = gpfpdDataV[5];
        gpfpdData.Latitude = gpfpdDataV[6];
        gpfpdData.Longitude = gpfpdDataV[7];
        gpfpdData.Altitude = gpfpdDataV[8];
        gpfpdData.Ve = gpfpdDataV[9];
        gpfpdData.Vn = gpfpdDataV[10];
        gpfpdData.Vu = gpfpdDataV[11];
        gpfpdData.Baseline = gpfpdDataV[12];
        gpfpdData.NSV1 = gpfpdDataV[13];
        gpfpdData.NSV2 = gpfpdDataV[14];
        std::cout << "GPFPD: " << gpfpdData.GPSTime << " ," << gpfpdData.GPSWeek << " ,"  << gpfpdData.Heading << " ,"  << gpfpdData.Pitch << " ,"  << gpfpdData.Roll << " ,"  << gpfpdData.Latitude << " ,"  << gpfpdData.Longitude << " ,"  << gpfpdData.Altitude <<  " ,"  << gpfpdData.Ve  << " ,"  << gpfpdData.Vn  << " ,"  << gpfpdData.Vu <<  " ,"  << gpfpdData.Baseline <<  " ,"  << gpfpdData.NSV1 <<  " ,"  << gpfpdData.NSV2  << std::endl;
        std::cout << "RTK status: " << gpfpdDataV[15] << std::endl;
    }
    else if (gpfpdDataV.size() < GPFPD_STRUCT_SIZE && gpfpdDataV.size() >= GPFPD_STRUCT_PART_SIZE) 
    {
        gpfpdData.GPSWeek = gpfpdDataV[1];
        gpfpdData.GPSTime = gpfpdDataV[2];
        gpfpdData.Heading = gpfpdDataV[3];
        gpfpdData.Pitch = gpfpdDataV[4];
        gpfpdData.Roll = gpfpdDataV[5];
        gpfpdData.Latitude = gpfpdDataV[6];
        gpfpdData.Longitude = gpfpdDataV[7];
        gpfpdData.Altitude = gpfpdDataV[8];
        gpfpdData.Ve = -1;
        gpfpdData.Vn = gpfpdDataV[9];
        gpfpdData.Vu = -1;
        gpfpdData.Baseline = -1;
        gpfpdData.NSV1 = -1;
        gpfpdData.NSV2 = -1;
        std::cout << "GPFPD: " << gpfpdData.GPSTime << " ," << gpfpdData.GPSWeek << " ,"  << gpfpdData.Heading << " ,"  << gpfpdData.Pitch << " ,"  << gpfpdData.Roll << " ,"  << gpfpdData.Latitude << " ,"  << gpfpdData.Longitude << " ,"  << gpfpdData.Altitude <<  " ,"  << gpfpdData.Ve  << " ,"  << gpfpdData.Vn  << " ,"  << gpfpdData.Vu <<  " ,"  << gpfpdData.Baseline <<  " ,"  << gpfpdData.NSV1 <<  " ,"  << gpfpdData.NSV2  << std::endl;
    }
    else 
    {
        printf("imcomplete data\n");
    }
}


void GPSIMU::setGTIMU() 
{
    if (gtimuDataV.size() == GTIMU_STRUCT_SIZE) 
    {
        gtimuData.GPSTime = gtimuDataV[0];
        gtimuData.GPSWeek = gtimuDataV[1];
        gtimuData.GyroX = gtimuDataV[2];
        gtimuData.GyroY = gtimuDataV[3];
        gtimuData.GyroZ = gtimuDataV[4];
        gtimuData.AccX = gtimuDataV[5];
        gtimuData.AccY = gtimuDataV[6];
        gtimuData.AccZ = gtimuDataV[7];
        std::cout << "GTIMU: " << gtimuData.GPSTime << " ," << gtimuData.GPSWeek << " ,"  << gtimuData.GyroX << " ,"  << gtimuData.GyroY << " ,"  << gtimuData.GyroZ << " ,"  << gtimuData.AccX << " ,"  << gtimuData.AccY << " ,"  << gtimuData.AccZ << std::endl;
    }
    else 
    {
        printf("imcomplete data\n");
    }
}


struct GPFPD GPSIMU::getGPFPDStruct() 
{
    return gpfpdData;
}


struct GTIMU GPSIMU::getGTIMUStruct() 
{
    return gtimuData;
}