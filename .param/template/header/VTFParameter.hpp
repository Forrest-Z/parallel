
/**
 * Created by yarten. 2018-1-25. snowing.
 * This file is automatically created by noxcreate, please don't modify.
 */

#pragma once

#include <nox>
#include <yaml-cpp/yaml.h>
#include <string>
#include <sstream>
#include <vector>
using std::vector;
using std::stringstream;
using std::string;
using namespace YAML;

namespace nox::parameter
{
    class VTFParameter : public nox::file::Parameter
    {
    public:

        double lsmax;
        double lsmin;
        double lsmulti;
        double eymax_min;
        double eymax_1;
        double eymax_2;
        double eymax_speed;
        double eymax_low_speed;
        double kappa_min;
        double wrmax;
        struct
        {
            int Steer_comp;
            int R_road;
            int Kappa_filter;
        } Option;


    public:

        bool Read(const string & dir) override
        {
            Node node = YAML::LoadFile(dir);

            lsmax = node["lsmax"].as<double>();
            lsmin = node["lsmin"].as<double>();
            lsmulti = node["lsmulti"].as<double>();
            eymax_min = node["eymax_min"].as<double>();
            eymax_1 = node["eymax_1"].as<double>();
            eymax_2 = node["eymax_2"].as<double>();
            eymax_speed = node["eymax_speed"].as<double>();
            eymax_low_speed = node["eymax_low_speed"].as<double>();
            kappa_min = node["kappa_min"].as<double>();
            wrmax = node["wrmax"].as<double>();

            {
                const Node & tmp = node["Option"];
                const Node & node = tmp;

                Option.Steer_comp = node["Steer_comp"].as<int>();
                Option.R_road = node["R_road"].as<int>();
                Option.Kappa_filter = node["Kappa_filter"].as<int>();
            }


            return true;
        }

        operator string() override
        {
            stringstream stream;

            stream << " lsmax: " << lsmax << "\n";
            stream << " lsmin: " << lsmin << "\n";
            stream << " lsmulti: " << lsmulti << "\n";
            stream << " eymax_min: " << eymax_min << "\n";
            stream << " eymax_1: " << eymax_1 << "\n";
            stream << " eymax_2: " << eymax_2 << "\n";
            stream << " eymax_speed: " << eymax_speed << "\n";
            stream << " eymax_low_speed: " << eymax_low_speed << "\n";
            stream << " kappa_min: " << kappa_min << "\n";
            stream << " wrmax: " << wrmax << "\n";
            stream << " Option:\n";
            stream << "   Steer_comp: " << Option.Steer_comp << "\n";
            stream << "   R_road: " << Option.R_road << "\n";
            stream << "   Kappa_filter: " << Option.Kappa_filter << "\n";

            return stream.str();
        }
    };
}