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

#include <rcutils/shared_library.h>

namespace performance_test {

std::string get_env_var(const char * env_var);

std::list<std::string> split(const std::string & value, const char delimiter);

bool is_file_exist(const char * filename);

std::string find_library_path(const std::string & library_name);

std::string get_library_name(std::string msg_type);

rcutils_shared_library_t get_library(std::string msg_type);

}
