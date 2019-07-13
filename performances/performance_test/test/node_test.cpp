/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include "performance_test/ros2/node.hpp"
#include "performance_test_msgs/msg/stamped10b.hpp"
#include "performance_test_msgs/srv/stamped10b.hpp"


int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



TEST(NodeTest, NodeConstructorTest)
{
  rclcpp::init(0, nullptr);

  std::string ros2_namespace = "node_namespace";
  bool use_ipc = true;

  auto node = std::make_shared<performance_test::Node>("node_name", ros2_namespace, use_ipc);

  auto trackers_vector_ptr = node->all_trackers();

  ASSERT_EQ((size_t)0, trackers_vector_ptr->size());

  rclcpp::shutdown();

}


TEST(NodeTest, NodeAddItemsTest)
{
  rclcpp::init(0, nullptr);

  auto topic = performance_test::Topic<performance_test_msgs::msg::Stamped10b>("my_topic");
  auto service = performance_test::Topic<performance_test_msgs::srv::Stamped10b>("my_service");

  auto node = std::make_shared<performance_test::Node>("node_name");

  node->add_subscriber(topic, PASS_BY_SHARED_PTR);
  node->add_periodic_publisher(topic, std::chrono::milliseconds(10), PASS_BY_UNIQUE_PTR);
  node->add_server(service);
  node->add_periodic_client(service, std::chrono::milliseconds(10));

  ASSERT_EQ((size_t)3, node->all_trackers()->size());

  rclcpp::shutdown();
}
