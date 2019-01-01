
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
    class VehicleParameter : public nox::file::Parameter
    {
    public:

        struct
        {
            double Width;
            double Height;
            double Length;
            double La;
            double Lb;
            double Weight;
            double WheelBase;
            struct
            {
                double Front;
                double Rear;
            } Cornering;

        } Physical;

        struct
        {
            double Max;
            double Min;
            double Ratio;
            double Offset;
        } Steering;

        double SpeedLimit;
        double BaseSpeed;

    public:

        bool Read(const string & dir) override
        {
            Node node = YAML::LoadFile(dir);


            {
                const Node & tmp = node["Physical"];
                const Node & node = tmp;

                Physical.Width = node["Width"].as<double>();
                Physical.Height = node["Height"].as<double>();
                Physical.Length = node["Length"].as<double>();
                Physical.La = node["La"].as<double>();
                Physical.Lb = node["Lb"].as<double>();
                Physical.Weight = node["Weight"].as<double>();
                Physical.WheelBase = node["WheelBase"].as<double>();

                {
                    const Node & tmp = node["Cornering"];
                    const Node & node = tmp;

                    Physical.Cornering.Front = node["Front"].as<double>();
                    Physical.Cornering.Rear = node["Rear"].as<double>();
                }

            }


            {
                const Node & tmp = node["Steering"];
                const Node & node = tmp;

                Steering.Max = node["Max"].as<double>();
                Steering.Min = node["Min"].as<double>();
                Steering.Ratio = node["Ratio"].as<double>();
                Steering.Offset = node["Offset"].as<double>();
            }

            SpeedLimit = node["SpeedLimit"].as<double>();
            BaseSpeed = node["BaseSpeed"].as<double>();

            return true;
        }

        operator string() override
        {
            stringstream stream;

            stream << " Physical:\n";
            stream << "   Width: " << Physical.Width << "\n";
            stream << "   Height: " << Physical.Height << "\n";
            stream << "   Length: " << Physical.Length << "\n";
            stream << "   La: " << Physical.La << "\n";
            stream << "   Lb: " << Physical.Lb << "\n";
            stream << "   Weight: " << Physical.Weight << "\n";
            stream << "   WheelBase: " << Physical.WheelBase << "\n";
            stream << "   Cornering:\n";
            stream << "     Front: " << Physical.Cornering.Front << "\n";
            stream << "     Rear: " << Physical.Cornering.Rear << "\n";
            stream << " Steering:\n";
            stream << "   Max: " << Steering.Max << "\n";
            stream << "   Min: " << Steering.Min << "\n";
            stream << "   Ratio: " << Steering.Ratio << "\n";
            stream << "   Offset: " << Steering.Offset << "\n";
            stream << " SpeedLimit: " << SpeedLimit << "\n";
            stream << " BaseSpeed: " << BaseSpeed << "\n";

            return stream.str();
        }
    };
}