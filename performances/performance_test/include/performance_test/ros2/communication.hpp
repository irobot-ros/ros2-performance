/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <string>

namespace performance_test {

template <typename Interface>
class CommunicationAbstraction
{
public:
  CommunicationAbstraction() = delete;
  CommunicationAbstraction(const std::string n)
    : name(n){};
  const std::string name;
};

// create alias templates for Topic and Service from common class CommunicationAbstraction

template <typename Msg>
using Topic = CommunicationAbstraction<Msg>;

template <typename Srv>
using Service = CommunicationAbstraction<Srv>;

}