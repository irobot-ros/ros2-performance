/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include <chrono>

#include "performance_metrics/dummy_work.hpp"

TEST(DummyWork, DummyWorkTest)
{
  using namespace std::chrono_literals;

  auto t1 = std::chrono::high_resolution_clock::now();
  performance_metrics::dummy_work(0us);

  auto t2 = std::chrono::high_resolution_clock::now();
  performance_metrics::dummy_work(5ms);

  auto t3 = std::chrono::high_resolution_clock::now();
  performance_metrics::dummy_work(100ms);

  auto t4 = std::chrono::high_resolution_clock::now();

  auto first_run_duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  auto second_run_duration = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
  auto third_run_duration = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);

  EXPECT_GT(second_run_duration.count(), first_run_duration.count());
  EXPECT_GT(third_run_duration.count(), second_run_duration.count());
}
