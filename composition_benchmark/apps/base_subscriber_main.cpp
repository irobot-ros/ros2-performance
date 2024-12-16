/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include "composition_benchmark/base_node.hpp"
#include "composition_benchmark/helpers/helper_options.hpp"
#include "composition_benchmark/helpers/helper_types.hpp"
#include "performance_metrics/stat_logger.hpp"
#include "performance_test/system.hpp"
#include "performance_test/utils/node_options.hpp"
#include "performance_test_factory/factory.hpp"

static
std::vector<IRobotNodePtr> create_subscriber_nodes(int argc, char ** argv)
{
  auto cli_options = CompositionOptions(argc, argv);
  auto node_options = performance_test::create_node_options(*cli_options.name);

  IRobotNodePtr node = std::make_shared<BaseNode>(node_options);
  const bool ipc = false;
  const bool ros_params = true;
  auto factory = performance_test_factory::TemplateFactory(
    ipc,
    ros_params);
  factory.add_subscriber_from_strings(
    node,
    *cli_options.msg_type,
    "my_topic",
    performance_metrics::Tracker::Options(),
    *cli_options.msg_pass_by);

  return {node};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto nodes = create_subscriber_nodes(argc, argv);
  auto system = std::make_unique<performance_test::System>(
    performance_test::ExecutorType::SINGLE_THREADED_EXECUTOR,
    performance_test::SpinType::SPIN);

  system->add_nodes(nodes);
  system->spin(MAX_HOURS, false);

  rclcpp::shutdown();
}
