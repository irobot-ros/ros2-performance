/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test_msgs/msg/stamped10b.hpp"
#include "performance_test_msgs/srv/stamped10b.hpp"



int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SystemTest, SystemAddNodesTest)
{
    rclcpp::init(0, nullptr);

    auto node_1 = std::make_shared<performance_test::Node>("node_1");
    auto node_2 = std::make_shared<performance_test::Node>("node_2");
    auto node_3 = std::make_shared<performance_test::Node>("node_3");
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vec = {node_2, node_3};

    int executors = 0;
    performance_test::System separate_threads_system(executors);

    separate_threads_system.add_node(node_1);
    separate_threads_system.add_node(nodes_vec);

    executors = 1;
    performance_test::System single_executor_system(executors);

    single_executor_system.add_node(node_1);
    single_executor_system.add_node(nodes_vec);

    rclcpp::shutdown();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}



TEST(SystemTest, SystemPubSubTest)
{
    rclcpp::init(0, nullptr);

    auto topic = performance_test::Topic<performance_test_msgs::msg::Stamped10b>("my_topic");

    int duration_sec = 1;
    performance_test::System ros2_system;

    // Create 1 pulisher node and 1 subscriber node
    auto pub_node = std::make_shared<performance_test::Node>("pub_node");
    pub_node->add_periodic_publisher(topic, 10ms, "unique_ptr", rmw_qos_profile_default);
    ros2_system.add_node(pub_node);

    auto sub_node = std::make_shared<performance_test::Node>("sub_node");
    sub_node->add_subscriber(topic, performance_test::Tracker::TrackingOptions(), rmw_qos_profile_default);
    ros2_system.add_node(sub_node);

    ros2_system.spin(duration_sec);

    rclcpp::shutdown();

    auto trackers_vec_ptr = sub_node->all_trackers();
    auto tracker = (*trackers_vec_ptr)[0];

    ASSERT_EQ("my_topic", tracker.first);
    ASSERT_GT(tracker.second.received(), (unsigned long int)1);


    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


TEST(SystemTest, SystemClientServerTest)
{
    rclcpp::init(0, nullptr);

    auto service = performance_test::Topic<performance_test_msgs::srv::Stamped10b>("my_service");

    int duration_sec = 2;
    performance_test::System ros2_system;

    // Create 1 client node and 1 server node
    auto client_node = std::make_shared<performance_test::Node>("client_node");
    client_node->add_periodic_client(service, 10ms, rmw_qos_profile_default);
    ros2_system.add_node(client_node);

    auto server_node = std::make_shared<performance_test::Node>("server_node");
    server_node->add_server(service, rmw_qos_profile_default);
    ros2_system.add_node(server_node);

    // discovery check does not work with client/server yet
    ros2_system.spin(duration_sec, false);

    rclcpp::shutdown();

    auto trackers_vec_ptr = client_node->all_trackers();
    auto tracker = (*trackers_vec_ptr)[0];

    ASSERT_EQ("my_service", tracker.first);
    ASSERT_GT(tracker.second.received(), (unsigned long int)1);


    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


TEST(SystemTest, SystemDifferentQoSTest)
{
    rclcpp::init(0, nullptr);

    auto topic = performance_test::Topic<performance_test_msgs::msg::Stamped10b>("my_topic");

    int duration_sec = 1;
    performance_test::System ros2_system;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    // Create 1 pulisher node and 1 subscriber node
    auto pub_node = std::make_shared<performance_test::Node>("pub_node");
    qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    pub_node->add_periodic_publisher(topic, 10ms, "unique_ptr", qos_profile);
    ros2_system.add_node(pub_node);

    auto sub_node = std::make_shared<performance_test::Node>("sub_node");
    qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_node->add_subscriber(topic, performance_test::Tracker::TrackingOptions(), qos_profile);
    ros2_system.add_node(sub_node);

    ros2_system.spin(duration_sec);

    rclcpp::shutdown();

    auto trackers_vec_ptr = sub_node->all_trackers();
    auto tracker = (*trackers_vec_ptr)[0];

    // they have incompatible qos so they shouldn't communicate
    ASSERT_EQ(tracker.second.received(), (unsigned long int)0);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
