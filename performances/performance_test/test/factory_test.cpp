
#include <gtest/gtest.h>

#include "performance_test/ros2/template_factory.hpp"



int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


TEST(FactoryTest, FactoryConstructorTest)
{
  performance_test::TemplateFactory factory;
}

TEST(FactoryTest, FactoryCreateFromStringTest)
{
  rclcpp::init(0, nullptr);

  performance_test::TemplateFactory factory;

  auto node = std::make_shared<performance_test::Node>("node_name");

  factory.add_subscriber_from_strings(node, "10b", "my_topic", performance_test::Tracker::TrackingOptions());
  factory.add_periodic_publisher_from_strings(node, "10b", "my_topic");
  factory.add_server_from_strings(node, "10b", "my_service");
  factory.add_periodic_client_from_strings(node, "10b", "my_service");

  ASSERT_EQ((size_t)2, node->all_trackers(false)->size());
  ASSERT_EQ((size_t)4, node->all_trackers(true)->size());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  rclcpp::shutdown();
}



