//
// Created by yarten on 18-5-15.
//

#pragma once

#include "convert_coordinates.hpp"

class CoordinateConverter
{
public:

    /**
     * 为了让（x, y）数值较小，可以设置一个经纬度为原点
     * @param longitude 本地的经度
     * @param latitude 本地的纬度
     */
    static void Setting(double longitude, double latitude);

    template <typename double_type>
    static void ll2xy(double lat, double lon, double_type & x, double_type & y)
    {
        double tx, ty;
        convert_coordinates::latlon_to_mercator(lat, lon, _scale, tx, ty);
        x = tx - _x0;
        y = ty - _y0;
    }

    template <typename double_type>
    static void xy2ll(double x, double y, double_type & longitude, double_type & latitude)
    {
        double lon, lat;
        convert_coordinates::mercator_to_latlon(x + _x0, y + _y0, _scale, lat, lon);
        longitude = lon;
        latitude = lat;
    }

private:

    static double _scale;
    static double _x0, _y0;
};