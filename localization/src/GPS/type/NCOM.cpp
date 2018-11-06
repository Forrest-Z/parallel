//
// Created by yarten on 18-3-10.
//

#include <GPS/type/NCOM.h>
#include <cstring>
#include <nox>
#include <GPS/tool/CoordinateConverter.h>
#include <iomanip>
USING_NAMESPACE_NOX;

bool NCOM::Parse(const string &msg)
{
    if(msg.length() != sizeof(Packet))
        return false;

    memcpy(&packet, msg.c_str(), sizeof(Packet));
    uint8_t * ptr = (uint8_t*)&packet;

    if(packet.sync != 0xe7)
        return false;

    uint8_t chksum = 0;
    for (unsigned int i = 1; i < sizeof(Packet) - 1; i++)
        chksum += ptr[i];

    bool ok = chksum == packet.chksum3;

    if(ok) Transfer();

    return ok;
}

void NCOM::Transfer()
{
    if (packet.nav_status == 4) //lock mode
    {
        switch (packet.channel)
        {
            //region Channel 0
            case 0:
            {
                switch (packet.chan.chan0.position_mode)
                {
                    case MODE_DIFFERENTIAL:
                    case MODE_DIFFERENTIAL_PP:
                    case MODE_RTK_FLOAT:
                    case MODE_RTK_INTEGER:
                    case MODE_RTK_FLOAT_PP:
                    case MODE_RTK_INTEGER_PP:
                    case MODE_DOPLER_PP:
                    case MODE_SPS_PP:
                        gps_data.gps_sat.rtk_sat = STATUS_GBAS_FIX;
                        break;
                    case MODE_OMNISTAR_VBS:
                    case MODE_OMNISTAR_HP:
                    case MODE_OMNISTAR_XP:
                    case MODE_WAAS:
                    case MODE_CDGPS:
                        gps_data.gps_sat.rtk_sat = STATUS_SBAS_FIX;
                        break;
                    case MODE_SPS:
                        gps_data.gps_sat.rtk_sat = STATUS_FIX;
                        break;
                    case MODE_NONE:
                    case MODE_SEARCH:
                    case MODE_DOPLER:
                    case MODE_NO_DATA:
                    case MODE_BLANKED:
                    case MODE_NOT_RECOGNISED:
                    case MODE_UNKNOWN:
                    default:
                        gps_data.gps_sat.rtk_sat = STATUS_NO_FIX;
                        break;
                }
                if (isLogging)
                    Logger::D("NCOM") << "rtk_sat : " << gps_data.gps_sat.rtk_sat << endl;
            } break;
            //endregion

            //region Channel 3
            case 3:
            {
                if (packet.chan.chan3.age < 150)
                {
                    //region Valid
                    position_covariance[0] = SQUARE((double)packet.chan.chan3.acc_position_east * 1e-3);
                    position_covariance[1] = SQUARE((double)packet.chan.chan3.acc_position_north * 1e-3);
                    position_covariance[2] = SQUARE((double)packet.chan.chan3.acc_position_down * 1e-3);
                    gps_data.gps_sat.gps_cov.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
                    //endregion
                }
                else
                {
                    //region Invalid
                    gps_data.gps_sat.gps_cov.position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
                    //endregion
                }

                //region Logging
                if (isLogging)
                    Logger::D("NCOM") << "position_covariance_type : " << gps_data.gps_sat.gps_cov.position_covariance_type << endl
                                      << setprecision(10) << fixed
                                      << "acc_position_east : "  << position_covariance[0] << endl
                                      << "acc_position_north : " << position_covariance[1] << endl
                                      << "acc_position_down : "  << position_covariance[2] << endl;
                //endregion
            } break;
            //endregion

            //region Channel 4
            case 4:
            {
                if (packet.chan.chan4.age < 150)
                {
                    //region Valid
                    velocity_covariance[0] = SQUARE((double)packet.chan.chan4.acc_velocity_east * 1e-3);
                    velocity_covariance[1] = SQUARE((double)packet.chan.chan4.acc_velocity_north * 1e-3);
                    velocity_covariance[2] = SQUARE((double)packet.chan.chan4.acc_velocity_down * 1e-3);
                    gps_data.gps_sat.gps_cov.velocity_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
                    //endregion
                }
                else
                {
                    //region Invalid
                    gps_data.gps_sat.gps_cov.velocity_covariance_type = COVARIANCE_TYPE_UNKNOWN;
                    //endregion
                }

                //region Logging
                if (isLogging)
                    Logger::D("NCOM") << "velocity_covariance_type : " << gps_data.gps_sat.gps_cov.velocity_covariance_type << endl
                                      << setprecision(10) << fixed
                                      << "acc_velocity_east : "  << velocity_covariance[0] << endl
                                      << "acc_velocity_north : " << velocity_covariance[1] << endl
                                      << "acc_velocity_down : "  << velocity_covariance[2] << endl;
                //endregion
            } break;
            //endregion

            //region Channel 5
            case 5:
            {
                if (packet.chan.chan5.age < 150)
                {
                    //region Valid
                    orientation_covariance[0] = SQUARE((double)packet.chan.chan5.acc_roll * 1e-5);
                    orientation_covariance[1] = SQUARE((double)packet.chan.chan5.acc_pitch * 1e-5);
                    orientation_covariance[2] = SQUARE((double)packet.chan.chan5.acc_heading * 1e-5);
                    gps_data.gps_sat.gps_cov.orientation_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
                    //endregion
                }
                else
                {
                    //region Invalid
                    gps_data.gps_sat.gps_cov.orientation_covariance_type = COVARIANCE_TYPE_UNKNOWN;
                    //endregion
                }

                //region Logging
                if (isLogging)
                    Logger::D("NCOM") << "orientation_covariance_type : " << gps_data.gps_sat.gps_cov.orientation_covariance_type << endl
                                      << setprecision(10) << fixed
                                      << "acc_roll : " << orientation_covariance[0] << endl
                                      << "acc_pitch : " << orientation_covariance[1] << endl
                                      << "acc_heading : " << orientation_covariance[2] << endl;
                //endregion
            } break;
            //endregion

            case 23: case 27: case 37: case 48: break;
        } // End Of Switch

        //region 更新时间、经纬度、转角、速度、角速度、加速度等其他信息
        gps_data.gpfpdData.GPSTime = packet.time;

        gps_data.gpfpdData.Latitude = packet.latitude * (180.0 / M_PI);
        gps_data.gpfpdData.Longitude = packet.longitude * (180.0 / M_PI);
        gps_data.gpfpdData.Altitude = packet.altitude;

        gps_data.gpfpdData.Heading = Radian(2 * M_PI - packet.heading * -1e-6).Get<Degree>();
        gps_data.gpfpdData.Pitch = Radian(packet.pitch * 1e-6).Get<Degree>();
        gps_data.gpfpdData.Roll = Radian(packet.roll * 1e-6).Get<Degree>();


        gps_data.gpfpdData.Ve = packet.vel_east * 1e-4;
        gps_data.gpfpdData.Vn = packet.vel_north * 1e-4;
        gps_data.gpfpdData.Vu = packet.vel_down * 1e-4;

        gps_data.gpfpdData.Ax = packet.accel_x * 1e-5;
        gps_data.gpfpdData.Ay = packet.accel_y * 1e-5;
        gps_data.gpfpdData.Az = packet.accel_z * 1e-5;

        gps_data.gpfpdData.Wx = packet.gyro_x * 1e-4;
        gps_data.gpfpdData.Wy = packet.gyro_y * 1e-4;
        gps_data.gpfpdData.Wz = packet.gyro_z * 1e-4;
        //endregion

        //region Logging
        if (isLogging)
        {
            Logger::D("NCOM") << "Latitude : "  << setprecision(10) << fixed << gps_data.gpfpdData.Latitude  << endl;
            Logger::D("NCOM") << "Longitude : " << setprecision(10) << fixed << gps_data.gpfpdData.Longitude << endl;
            Logger::D("NCOM") << "Altitude : "  << setprecision(10) << fixed << gps_data.gpfpdData.Altitude  << endl;
            Logger::D("NCOM") << "Heading :"    << setprecision(10) << fixed << gps_data.gpfpdData.Heading   << endl;
            Logger::D("NCOM") << "Pitch : "     << setprecision(10) << fixed << gps_data.gpfpdData.Pitch     << endl;
            Logger::D("NCOM") << "Roll : "      << setprecision(10) << fixed << gps_data.gpfpdData.Roll      << endl;
            Logger::D("NCOM") << "Ve : "        << setprecision(10) << fixed << gps_data.gpfpdData.Ve        << endl;
            Logger::D("NCOM") << "Vn : "        << setprecision(10) << fixed << gps_data.gpfpdData.Vn        << endl;
            Logger::D("NCOM") << "Vu : "        << setprecision(10) << fixed << gps_data.gpfpdData.Vu        << endl;
            Logger::D("NCOM") << "Ax : "        << setprecision(10) << fixed << gps_data.gpfpdData.Ax        << endl;
            Logger::D("NCOM") << "Ay : "        << setprecision(10) << fixed << gps_data.gpfpdData.Ay        << endl;
            Logger::D("NCOM") << "Az : "        << setprecision(10) << fixed << gps_data.gpfpdData.Az        << endl;
            Logger::D("NCOM") << "Wx : "        << setprecision(10) << fixed << gps_data.gpfpdData.Wx        << endl;
            Logger::D("NCOM") << "Wy : "        << setprecision(10) << fixed << gps_data.gpfpdData.Wy        << endl;
            Logger::D("NCOM") << "Wz : "        << setprecision(10) << fixed << gps_data.gpfpdData.Wz        << endl;
        }
        //endregion
    }
}

nav_msgs::Odometry NCOM::Get()
{
    Odometry t;
    CoordinateConverter::ll2xy(gps_data.gpfpdData.Latitude, gps_data.gpfpdData.Longitude, t.pose.t.x, t.pose.t.y);
    t.pose.Set(
        Rotation
        (
            Degree(-gps_data.gpfpdData.Heading),
            Degree(gps_data.gpfpdData.Roll),
            Degree(gps_data.gpfpdData.Pitch)
        )
    );

    nav_msgs::Odometry data;
    t.To(data);
    return data;
}

//sensor_msgs::Imu NCOM::GetRaw()
//{
//    sensor_msgs::Imu data;
//    data.header.stamp.fromSec(gps_data.gpfpdData.GPSTime * 1e-3);
//    data.latitude = gps_data.gpfpdData.Latitude;
//    data.longitude = gps_data.gpfpdData.Longitude;
//    data.heading = gps_data.gpfpdData.Heading;
//    data.roll = gps_data.gpfpdData.Roll;
//    data.pitch = gps_data.gpfpdData.Pitch;
//    data.v.x = gps_data.gpfpdData.Ve;
//    data.v.y = gps_data.gpfpdData.Vn;
//    data.v.z = gps_data.gpfpdData.Vu;
//    data.w.x = gps_data.gpfpdData.Wx;
//    data.w.y = gps_data.gpfpdData.Wy;
//    data.w.z = gps_data.gpfpdData.Wz;
//    data.a.x = gps_data.gpfpdData.Ax;
//    data.a.y = gps_data.gpfpdData.Ay;
//    data.a.z = gps_data.gpfpdData.Az;
//    data.state = GetState();
//    return data;
//}

GPFPD NCOM::GetGPGGAData()
{
    return gps_data.gpfpdData;
}

int NCOM::GetState()
{
    if(position_covariance[0] < 0.3 && position_covariance[1] < 0.3)
    {
        if(gps_data.gps_sat.rtk_sat >= RTK_STATUS::STATUS_SBAS_FIX)
            return 0;
        else return 1;
    }
    else return 2;
}

double NCOM::SQUARE(double x)
{
    return x * x;
}

void NCOM::EnableLogging(bool enable)
{
    isLogging = enable;
}

NCOM::NCOM()
{
    isLogging = false;
}
