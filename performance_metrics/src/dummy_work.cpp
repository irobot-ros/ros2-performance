/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <chrono>

#include "performance_metrics/dummy_work.hpp"

namespace performance_metrics
{

void dummy_work(std::chrono::microseconds work_duration)
{
  if (work_duration == std::chrono::microseconds::zero()) {
    return;
  }

  // Do dummy work until we reach required duration
  auto start_time = std::chrono::high_resolution_clock::now();
  int dummy_result = 0;
  while (std::chrono::high_resolution_clock::now() - start_time < work_duration) {
    for (int i = 1; i < 100; i++) {
      for (int k = 1; k < i; k++) {
        if (i % k == 0) {
          dummy_result++;
        }
      }
    }
  }
}

}  // namespace performance_metrics
