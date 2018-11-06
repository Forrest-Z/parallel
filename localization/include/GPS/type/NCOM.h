//
// Created by yarten on 18-3-10.
//
#pragma once

#include <stdint.h>
#include <string>
#include "GPSIMU.h"
#include <nox>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

using std::string;

class NCOM
{
public:

#pragma pack(push, 1)
    typedef union
    {
        uint8_t bytes[8];
        struct {
            uint32_t :32;
            uint8_t num_sats;
            uint8_t position_mode;
            uint8_t velocity_mode;
            uint8_t orientation_mode;
        } chan0;
        struct {
            uint16_t acc_position_north; // 1e-3 m
            uint16_t acc_position_east;
            uint16_t acc_position_down;
            uint8_t age;
        } chan3;
        struct {
            uint16_t acc_velocity_north; // 1e-3 m/s
            uint16_t acc_velocity_east;
            uint16_t acc_velocity_down;
            uint8_t age;
        } chan4;
        struct {
            uint16_t acc_heading; // 1e-5 rad
            uint16_t acc_pitch;
            uint16_t acc_roll;
            uint8_t age;
        } chan5;
        struct {
            int16_t base_station_age;
            int8_t base_station_id[4];
        } chan20;
        struct {
            uint16_t delay_ms;
        } chan23;
        struct {
            uint8_t heading_quality; // 0:None 1:Poor 2:RTK Float 3:RTK Integer
        } chan27;
        struct {
            int16_t heading_misalignment_angle; // 1e-4 rad
            uint16_t heading_misalignment_accuracy; // 1e-4 rad
            uint16_t :16;
            uint8_t valid;
        } chan37;
        struct {
            int16_t undulation; // 5e-3 m
            uint8_t HDOP; // 1e-1
            uint8_t PDOP; // 1e-1
        } chan48;
    } Channel;

    typedef struct
    {
        uint8_t sync;
        uint16_t time;
        int32_t accel_x :24; // 1e-5 m/s^2
        int32_t accel_y :24; // 1e-5 m/s^2
        int32_t accel_z :24; // 1e-5 m/s^2
        int32_t gyro_x :24; // 1e-4 rad/s
        int32_t gyro_y :24; // 1e-4 rad/s
        int32_t gyro_z :24; // 1e-4 rad/s
        uint8_t nav_status;
        uint8_t chksum1;
        double latitude;
        double longitude;
        float  altitude;
        int32_t vel_north :24; // 1e-4 m/s
        int32_t vel_east :24; // 1e-4 m/s
        int32_t vel_down :24; // 1e-4 m/s
        int32_t heading :24; // 1e-6 rad
        int32_t pitch :24; // 1e-6 rad
        int32_t roll :24; // 1e-6 rad
        uint8_t chksum2;
        uint8_t channel;
        Channel chan;
        uint8_t chksum3;
    } Packet;

    enum
    {
        MODE_NONE = 0,
        MODE_SEARCH = 1,
        MODE_DOPLER = 2,
        MODE_SPS = 3,
        MODE_DIFFERENTIAL = 4,
        MODE_RTK_FLOAT = 5,
        MODE_RTK_INTEGER = 6,
        MODE_WAAS = 7,
        MODE_OMNISTAR_VBS = 8,
        MODE_OMNISTAR_HP = 9,
        MODE_NO_DATA = 10,
        MODE_BLANKED = 11,
        MODE_DOPLER_PP = 12,
        MODE_SPS_PP = 13,
        MODE_DIFFERENTIAL_PP = 14,
        MODE_RTK_FLOAT_PP = 15,
        MODE_RTK_INTEGER_PP = 16,
        MODE_OMNISTAR_XP = 17,
        MODE_CDGPS = 18,
        MODE_NOT_RECOGNISED = 19,
        MODE_UNKNOWN = 20,
    };
#pragma pack(pop)

public:

    enum RTK_STATUS
    {
        STATUS_NO_FIX=0,
        STATUS_FIX,
        STATUS_SBAS_FIX,
        STATUS_GBAS_FIX
    };

    enum COVARIANCE
    {
        COVARIANCE_TYPE_UNKNOWN = 0,
        COVARIANCE_TYPE_APPROXIMATED,
        COVARIANCE_TYPE_DIAGONAL_KNOWN,
        COVARIANCE_TYPE_KNOWN
    };

    struct GPS_COVARIANCE
    {
        COVARIANCE position_covariance_type;
        COVARIANCE velocity_covariance_type;
        COVARIANCE orientation_covariance_type;
    };

    struct GPS_SAT
    {
        RTK_STATUS rtk_sat;
        GPS_COVARIANCE gps_cov;
    };

    struct GPS_DATA
    {
        GPFPD gpfpdData;
        GPS_SAT gps_sat;
    };

    enum State
    {
        Normal = 0,
        NotSoGood = 1,
        Bad = 2
    };

    /**
     * 传入惯导NCOM模式的一帧数据包
     * @param msg
     * @return 是否解析成功（校验和通过）
     */
    bool Parse(const string & msg);

    void Transfer();

    nav_msgs::Odometry Get();

//    sensor_msgs::Imu GetRaw();

    GPFPD GetGPGGAData();

    int GetState();

    void EnableLogging(bool enable);

    inline double SQUARE(double x);

    NCOM();

private:

    double position_covariance[3];
    double velocity_covariance[3];
    double orientation_covariance[3];

    Packet packet;

    GPS_DATA gps_data;

    bool isLogging;
};