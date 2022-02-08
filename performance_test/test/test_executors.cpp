/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "performance_test/executors.hpp"

using namespace std::chrono_literals;

TEST(ExecutorsTest, SleepTaskTest)
{
  // This should return immediately
  performance_test::sleep_task(0s);

  // Wait for a small while
  performance_test::sleep_task(10ms);

  bool sleep_done = false;
  std::thread sleeping_thread([&sleep_done]() {
      rclcpp::init(0, nullptr);
      performance_test::sleep_task(1000s);
      sleep_done = true;
    });

  while (!rclcpp::ok()) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(rclcpp::ok());

  // This should return immediately
  performance_test::sleep_task(0s);

  // Wait for a small while
  performance_test::sleep_task(10ms);

  // Shutdown, this should also wake up the thread
  rclcpp::shutdown();
  sleeping_thread.join();
  EXPECT_TRUE(sleep_done);
}
