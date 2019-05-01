
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

  rclcpp::shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}


TEST(FactoryTest, FactoryCreateFromIndicesTest)
{
  rclcpp::init(0, nullptr);

  performance_test::TemplateFactory factory;

  int n_subscriber_nodes = 2;
  int n_publisher_nodes = 2;
  std::string msg_type = "10b";
  float frequency = 1;

  int subscriber_start_index = 0;
  int subscriber_end_index = n_subscriber_nodes;
  int publisher_start_index = n_subscriber_nodes;
  int publisher_end_index = n_subscriber_nodes + n_publisher_nodes;

  auto sub_nodes =
    factory.create_subscriber_nodes(
      subscriber_start_index,
      subscriber_end_index,
      n_publisher_nodes,
      msg_type);

  auto pub_nodes =
    factory.create_periodic_publisher_nodes(
        publisher_start_index,
        publisher_end_index,
        frequency,
        msg_type);


  ASSERT_EQ((size_t)2, sub_nodes.size());
  ASSERT_EQ((size_t)2, pub_nodes.size());

  for (const auto& n : sub_nodes){
    ASSERT_EQ((size_t)2, n->all_trackers()->size());
  }

  for (const auto& n : pub_nodes){
    ASSERT_EQ((size_t)1, n->all_trackers(true)->size());
  }

  rclcpp::shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}


TEST(FactoryTest, FactoryCreateFromJsonTest)
{
  rclcpp::init(0, nullptr);

  std::string this_file_path = __FILE__;
  std::string this_dir_path = this_file_path.substr(0, this_file_path.rfind("/"));
  std::string json_path = this_dir_path + std::string("/files/test_architecture.json");

  performance_test::TemplateFactory factory;

  auto nodes_vec = factory.parse_topology_from_json(json_path);

  ASSERT_EQ((size_t)3, nodes_vec.size());

  ASSERT_STREQ("node_0", nodes_vec[0]->get_name());
  ASSERT_STREQ("node_1", nodes_vec[1]->get_name());
  ASSERT_STREQ("node_2", nodes_vec[2]->get_name());

  ASSERT_EQ((size_t)2, nodes_vec[0]->all_trackers(true)->size());
  ASSERT_EQ((size_t)2, nodes_vec[1]->all_trackers(true)->size());
  ASSERT_EQ((size_t)1, nodes_vec[2]->all_trackers(true)->size());

  rclcpp::shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
