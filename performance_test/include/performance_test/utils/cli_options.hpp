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
#include <vector>
#include "performance_test/tracker.hpp"

namespace performance_test {

class Options
{
public:
  Options();

  Options(int argc, char** argv);

  void parse(int argc, char** argv);

  bool ipc;
  bool ros_params;
  bool name_threads;
  int executor;
  int node;
  int duration_sec;
  int resources_sampling_per_ms;
  std::vector<std::string> topology_json_list;
  performance_test::Tracker::TrackingOptions tracking_options;
};

std::ostream &operator<<(std::ostream &os, const Options &options);

}
