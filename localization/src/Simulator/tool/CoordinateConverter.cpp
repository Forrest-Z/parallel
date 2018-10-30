//
// Created by yarten on 18-5-15.
//

#include <Simulator/tool/CoordinateConverter.h>
#include <Simulator/tool/convert_coordinates.hpp>

double CoordinateConverter::_scale = 0;
double CoordinateConverter::_x0 = 0;
double CoordinateConverter::_y0 = 0;

void CoordinateConverter::Setting(double longitude, double latitude)
{
    _scale = convert_coordinates::lat_to_scale(latitude);
    convert_coordinates::latlon_to_mercator(latitude, longitude, _scale, _x0, _y0);
}

