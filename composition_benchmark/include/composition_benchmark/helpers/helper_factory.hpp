#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/global_factory.hpp>
#include <composition_benchmark/helpers/helper_types.hpp>

void global_factory_generic_setup(int argc, char** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  std::vector<global_factory::NodeArguments> node_arguments;
  for (size_t i = 1; i < non_ros_args.size(); i ++) {
    global_factory::NodeArguments args;
    args.name = non_ros_args[i];
    node_arguments.push_back(args);
  }

  global_factory::setup_factory(node_arguments);
}

template<typename NodeT>
std::vector<IRobotNodePtr>
global_factory_create_generic_nodes(int argc, char** argv)
{
  global_factory_generic_setup(argc, argv);

  size_t num_nodes = global_factory::get_num_registered_nodes();
  std::vector<IRobotNodePtr> nodes;
  for (size_t i = 0; i < num_nodes; i++) {
    IRobotNodePtr node = std::make_shared<NodeT>();
    nodes.push_back(node);
  }

  return nodes;
}
