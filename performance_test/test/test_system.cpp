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
#include <vector>

#include "performance_test/system.hpp"
#include "performance_test/performance_node.hpp"
#include "performance_test_msgs/msg/sample.hpp"
#include "performance_test_msgs/srv/sample.hpp"

class TestSystem : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestSystem, SystemAddNodesTest)
{
  auto node_1 =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("node_1");
  auto node_2 =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("node_2");
  auto node_3 =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("node_3");
  std::vector<std::shared_ptr<performance_test::PerformanceNode<rclcpp::Node>>> nodes_vec =
  {node_2, node_3};

  auto system_executor = performance_test::ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR;
  performance_test::System system(system_executor);

  system.add_node(node_1);
  system.add_nodes(nodes_vec);
}

TEST_F(TestSystem, SystemPubSubTest)
{
  auto duration_sec = std::chrono::seconds(1);
  auto system_executor = performance_test::ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR;
  performance_test::System ros2_system(system_executor);

  // Create 1 pulisher node and 1 subscriber node
  auto pub_node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("pub_node");
  pub_node->add_periodic_publisher<performance_test_msgs::msg::Sample>(
    "my_topic",
    std::chrono::milliseconds(10),
    performance_test::msg_pass_by_t::PASS_BY_UNIQUE_PTR,
    rclcpp::SensorDataQoS());
  ros2_system.add_node(pub_node);

  auto sub_node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("sub_node");
  sub_node->add_subscriber<performance_test_msgs::msg::Sample>(
    "my_topic",
    performance_test::msg_pass_by_t::PASS_BY_SHARED_PTR,
    performance_metrics::Tracker::Options(),
    rclcpp::SensorDataQoS());
  ros2_system.add_node(sub_node);

  ros2_system.spin(duration_sec);

  auto tracker = sub_node->sub_trackers()[0];

  ASSERT_EQ("sub_node", tracker.get_node_name());
  ASSERT_EQ("my_topic", tracker.get_entity_name());
  ASSERT_GT(tracker.received(), (uint64_t)1);
}

TEST_F(TestSystem, SystemClientServerTest)
{
  auto duration_sec = std::chrono::seconds(2);
  auto system_executor = performance_test::ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR;
  performance_test::System ros2_system(system_executor);

  // Create 1 client node and 1 server node
  auto client_node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("client_node");
  client_node->add_periodic_client<performance_test_msgs::srv::Sample>(
    "my_service",
    std::chrono::milliseconds(10),
    rclcpp::ServicesQoS());
  ros2_system.add_node(client_node);

  auto server_node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("server_node");
  server_node->add_server<performance_test_msgs::srv::Sample>(
    "my_service",
    rclcpp::ServicesQoS());
  ros2_system.add_node(server_node);

  // discovery check does not work with client/server yet
  ros2_system.spin(duration_sec, false);

  auto tracker = client_node->client_trackers()[0];

  ASSERT_EQ("client_node", tracker.get_node_name());
  ASSERT_EQ("my_service", tracker.get_entity_name());
  ASSERT_GT(tracker.received(), (uint64_t)1);
}

TEST_F(TestSystem, SystemDifferentQoSTest)
{
  auto duration_sec = std::chrono::seconds(1);
  auto system_executor = performance_test::ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR;
  performance_test::System ros2_system(system_executor);
  auto qos_profile = rclcpp::QoS(10);

  // Create 1 pulisher node and 1 subscriber node
  auto pub_node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("pub_node");
  qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  pub_node->add_periodic_publisher<performance_test_msgs::msg::Sample>(
    "my_topic",
    std::chrono::milliseconds(10),
    performance_test::msg_pass_by_t::PASS_BY_UNIQUE_PTR,
    qos_profile);
  ros2_system.add_node(pub_node);

  auto sub_node =
    std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>("sub_node");
  qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  sub_node->add_subscriber<performance_test_msgs::msg::Sample>(
    "my_topic",
    performance_test::msg_pass_by_t::PASS_BY_SHARED_PTR,
    performance_metrics::Tracker::Options(),
    qos_profile);
  ros2_system.add_node(sub_node);

  ros2_system.spin(duration_sec);

  auto tracker = sub_node->sub_trackers()[0];

  // they have incompatible qos so they shouldn't communicate
  ASSERT_EQ(tracker.received(), (uint64_t)0);
}
