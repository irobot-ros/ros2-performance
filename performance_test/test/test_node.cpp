/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "performance_test/performance_node.hpp"
#include "performance_test_msgs/msg/sample.hpp"
#include "performance_test_msgs/srv/sample.hpp"

class TestNode : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestNode, NodeConstructorTest)
{
  std::string ros2_namespace = "node_namespace";

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(true);
  node_options.start_parameter_event_publisher(true);

  auto node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>(
    "node_name",
    ros2_namespace,
    node_options);

  auto trackers = node->sub_and_client_trackers();

  ASSERT_EQ(static_cast<size_t>(0), trackers.size());
}

TEST_F(TestNode, NodeAddItemsTest)
{
  auto node = std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("node_name");

  node->add_subscriber<performance_test_msgs::msg::Sample>(
    "my_topic",
    PASS_BY_SHARED_PTR);
  node->add_periodic_publisher<performance_test_msgs::msg::Sample>(
    "my_topic",
    std::chrono::milliseconds(10),
    PASS_BY_UNIQUE_PTR);
  node->add_server<performance_test_msgs::srv::Sample>(
    "my_service");
  node->add_periodic_client<performance_test_msgs::srv::Sample>(
    "my_service",
    std::chrono::milliseconds(10));

  ASSERT_EQ(static_cast<size_t>(2), node->sub_and_client_trackers().size());
}

TEST_F(TestNode, LifecycleNodeConstructorTest)
{
  std::string ros2_namespace = "node_namespace";

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(true);
  node_options.start_parameter_event_publisher(true);

  auto node =
    std::make_shared<performance_test::PerformanceNode<rclcpp_lifecycle::LifecycleNode>>(
    "node_name",
    ros2_namespace,
    node_options);

  auto trackers = node->sub_and_client_trackers();

  ASSERT_EQ(static_cast<size_t>(0), trackers.size());
}

TEST_F(TestNode, LifecycleNodeAddItemsTest)
{
  auto node =
    std::make_shared<performance_test::PerformanceNode<rclcpp_lifecycle::LifecycleNode>>(
    "node_name");

  node->add_subscriber<performance_test_msgs::msg::Sample>(
    "my_topic",
    PASS_BY_SHARED_PTR);
  node->add_periodic_publisher<performance_test_msgs::msg::Sample>(
    "my_topic",
    std::chrono::milliseconds(10),
    PASS_BY_UNIQUE_PTR);
  node->add_server<performance_test_msgs::srv::Sample>(
    "my_service");
  node->add_periodic_client<performance_test_msgs::srv::Sample>(
    "my_service",
    std::chrono::milliseconds(10));

  ASSERT_EQ(static_cast<size_t>(2), node->sub_and_client_trackers().size());
}
