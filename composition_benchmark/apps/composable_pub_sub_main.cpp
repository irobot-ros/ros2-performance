/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <cstdlib>
#include <string>

#include "composition_benchmark/composable_publisher.hpp"
#include "composition_benchmark/composable_subscriber.hpp"
#include "composition_benchmark/helpers/helper_options.hpp"
#include "composition_benchmark/helpers/helper_types.hpp"
#include "performance_test/system.hpp"
#include "performance_test/utils/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

static
std::vector<IRobotNodePtr> create_pub_sub_system(int argc, char ** argv)
{
  auto options = CompositionOptions(argc, argv);

  bool isolated = false;
  int executor_id = 0;
  if (*options.spin_type == "spin_isolated") {
    isolated = true;
  }

  std::vector<IRobotNodePtr> nodes;

  std::vector<rclcpp::Parameter> pub_parameters = {
    {"executor_id", isolated ? executor_id++ : 0},
    {"topic", "topic"},
    {"frequency", *options.pub_frequency},
    {"size", *options.msg_size}
  };
  auto pub_options = performance_test::create_node_options("pub_node", "", pub_parameters);
  pub_options.use_intra_process_comms(*options.use_ipc);
  IRobotNodePtr pub_node = std::make_shared<ComposablePublisher>(pub_options);
  nodes.push_back(pub_node);

  for (size_t i = 0; i < *options.num_subs; i++) {
    std::string node_name = std::string("sub_node_") + std::to_string(i);
    std::vector<rclcpp::Parameter> sub_parameters = {
      {"executor_id", isolated ? executor_id++ : 0},
      {"topic", "topic"}
    };
    auto sub_options = performance_test::create_node_options(node_name, "", sub_parameters);
    sub_options.use_intra_process_comms(*options.use_ipc);
    IRobotNodePtr sub_node = std::make_shared<ComposableSubscriber>(sub_options);
    nodes.push_back(sub_node);
  }

  return nodes;
}

int main(int argc, char ** argv)
{
  auto options = CompositionOptions(argc, argv);

  performance_test::SpinType spin_type = performance_test::SpinType::SPIN;
  if (*options.spin_type == "spin_some") {
    spin_type = performance_test::SpinType::SPIN_SOME;
  }

  rclcpp::init(argc, argv);

  auto nodes = create_pub_sub_system(argc, argv);
  auto system = std::make_unique<performance_test::System>(
    performance_test::ExecutorType::SINGLE_THREADED_EXECUTOR,
    spin_type);

  system->add_nodes(nodes);
  system->spin(MAX_HOURS, false);

  rclcpp::shutdown();
}
