/**
 * @file Common.h 提供一些公共的常用操作
 */

#pragma once

#include <nox>

namespace nox::app
{
    void SendTF(const std::string & from, const std::string & to, const type::Pose & pose);
}