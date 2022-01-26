#pragma once

#include <vector>

#include <composition_benchmark/global_factory.hpp>
#include <performance_test/utils/cli_args.hpp>

void generic_factory_setup(int argc, char** argv)
{
  auto non_ros_args = performance_test::get_non_ros_args(argc, argv);

  std::vector<global_factory::NodeArguments> node_arguments;
  for (size_t i = 1; i < non_ros_args.size(); i ++) {
    global_factory::NodeArguments args;
    args.name = non_ros_args[i];
    node_arguments.push_back(args);
  }

  global_factory::setup_factory(node_arguments);
}
