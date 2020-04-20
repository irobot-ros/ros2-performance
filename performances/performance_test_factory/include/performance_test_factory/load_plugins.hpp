/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */


#pragma once

#include <memory>
#include <string>

#include <rcpputils/shared_library.hpp>

namespace performance_test {

std::shared_ptr<rcpputils::SharedLibrary> get_library(std::string msg_type);

}
