/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <cstddef>
#include <cstdio>
#include <cstring>

#include <list>

#include <fstream>
#include <iostream>
#include <sstream>

#include "performance_test_factory/load_plugins.hpp"

using namespace performance_test;

static std::string get_library_name(std::string msg_type)
{
  static const std::string DEFAULT_LIBRARY_NAME = "irobot_interfaces_plugin";
  static const std::string namespace_delimiter = "::";

  std::string library_name;
  size_t pos = msg_type.find(namespace_delimiter);
  if (pos == std::string::npos) {
    library_name = DEFAULT_LIBRARY_NAME + "_implementation";
  } else {
    library_name = msg_type.substr(0, pos) + "_implementation";
  }

  return library_name;
}

std::shared_ptr<rcpputils::SharedLibrary> performance_test::get_library(std::string msg_type)
{
  std::string library_name = get_library_name(msg_type);

  const std::string library_path = rcpputils::get_platform_library_name(library_name);
  auto library = std::make_shared<rcpputils::SharedLibrary>(library_path);

  return library;
}
