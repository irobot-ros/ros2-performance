/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef COMPOSITION_BENCHMARK__COMPOSABLE_SUBSCRIBER_HPP_
#define COMPOSITION_BENCHMARK__COMPOSABLE_SUBSCRIBER_HPP_

#include "performance_test/performance_node.hpp"

class ComposableSubscriber : public performance_test::PerformanceNode<rclcpp::Node>
{
public:
  explicit ComposableSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ComposableSubscriber();
};
