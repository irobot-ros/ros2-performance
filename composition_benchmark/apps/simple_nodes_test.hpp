#pragma once

#include <chrono>
#include <thread>
#include <composition_benchmark/global_factory.hpp>

#include <rclcpp/rclcpp.hpp>

template<typename NodeT>
void do_test(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  size_t non_ros_argc = non_ros_args_c_strings.size();
  char** non_ros_argv = non_ros_args_c_strings.data();

  std::vector<global_factory::NodeArguments> node_arguments;
  for (size_t i = 1; i < non_ros_argc; i ++) {
    global_factory::NodeArguments args;
    args.name = non_ros_argv[i];
    node_arguments.push_back(args);
  }

  global_factory::setup_nodes(node_arguments);

  std::vector<std::shared_ptr<NodeT>> nodes;
  for (size_t i = 0; i < node_arguments.size(); i++) {
    auto node = std::make_shared<NodeT>();
    nodes.push_back(node);
  }

  std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::hours(std::numeric_limits<int>::max()));

  rclcpp::shutdown();
}
