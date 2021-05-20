/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include "performance_test/ros2/lifecycle_node.hpp"
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

  auto node = std::make_shared<performance_test::LifecycleNode>("node_name", ros2_namespace, node_options);

  auto trackers_vector_ptr = node->all_trackers();

  ASSERT_EQ((size_t)0, trackers_vector_ptr->size());
}


TEST_F(TestNode, NodeAddItemsTest)
{
  auto topic = performance_test::Topic<performance_test_msgs::msg::Sample>("my_topic");
  auto service = performance_test::Topic<performance_test_msgs::srv::Sample>("my_service");

  auto node = std::make_shared<performance_test::LifecycleNode>("node_name");

  node->add_subscriber(topic, PASS_BY_SHARED_PTR);
  node->add_periodic_publisher(topic, std::chrono::milliseconds(10), PASS_BY_UNIQUE_PTR);
  node->add_server(service);
  node->add_periodic_client(service, std::chrono::milliseconds(10));

  ASSERT_EQ((size_t)3, node->all_trackers()->size());
}
