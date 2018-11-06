//
// Created by yarten on 18-5-15.
//

#include <GPS/tool/CoordinateConverter.h>


double CoordinateConverter::_x0 = 0;
double CoordinateConverter::_y0 = 0;

void CoordinateConverter::Setting(double x0, double y0)
{
    _x0 = x0;
    _y0 = y0;
}

void CoordinateConverter::ll2xy(double lat, double lon, double &x, double &y)
{

}

void CoordinateConverter::xy2ll(double x, double y, double &lon, double &lat)
{

}

