//
// Created by yarten on 18-5-15.
//

#pragma once

class CoordinateConverter
{
public:

    /**
     * 为了让（x, y）数值较小，可以设置一个经纬度为原点
     * @param origin_x 本地的经度
     * @param origin_y 本地的纬度
     */
    static void Setting(double origin_x, double origin_y);

    static void ll2xy(double lat, double lon, double & x, double & y);

    static void xy2ll(double x, double y, double & lon, double & lat);

private:

    static double _x0, _y0;
};