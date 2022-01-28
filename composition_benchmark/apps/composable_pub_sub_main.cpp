#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/composable_publisher.hpp>
#include <composition_benchmark/composable_subscriber.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>

std::vector<IRobotNodePtr> create_pub_sub_system(int argc, char** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  assert(non_ros_args.size() >= 5);
  size_t num_subs = atoi(non_ros_args[1].c_str());
  int pub_frequency = atoi(non_ros_args[2].c_str());
  int msg_size = atoi(non_ros_args[3].c_str());
  bool use_ipc = static_cast<bool>(atoi(non_ros_args[4].c_str()));

  std::vector<IRobotNodePtr> nodes;

  rclcpp::NodeOptions pub_options;
  pub_options.parameter_overrides({
    {"topic", "dummy_topic"},
    {"frequency", pub_frequency},
    {"size", msg_size}
  });
  pub_options.arguments({
    "--ros-args", "-r", "__node:=pub_node"
  });
  pub_options.use_intra_process_comms(use_ipc);
  IRobotNodePtr pub_node = std::make_shared<ComposablePublisher>(pub_options);
  nodes.push_back(pub_node);

  for (size_t i = 0; i < num_subs; i++) {
    std::string node_name_remap = std::string("__node:=") + std::string("sub_node_") + std::to_string(i);
    rclcpp::NodeOptions sub_options;
    sub_options.parameter_overrides({
      {"topic", "dummy_topic"}
    });
    sub_options.arguments({
      "--ros-args", "-r", node_name_remap
    });
    sub_options.use_intra_process_comms(use_ipc);
    IRobotNodePtr sub_node = std::make_shared<ComposableSubscriber>(sub_options);
    nodes.push_back(sub_node);
  }

  return nodes;
}

int main(int argc, char** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  assert(non_ros_args.size() >= 6);
  std::string spin_type = non_ros_args[5];

  run_func_t run_func;
  if (spin_type == "spin") {
    run_func = std::bind(spin_task, std::placeholders::_1, std::chrono::seconds(15));
  } else if (spin_type == "spin_future") {
    run_func = std::bind(spin_future_complete_task, std::placeholders::_1, std::chrono::seconds(15));
  } else if (spin_type == "spin_isolated") {
    run_func = std::bind(spin_isolated_task, std::placeholders::_1, std::chrono::seconds(15));
  } else if (spin_type == "spin_some") {
    run_func = std::bind(spin_some_task, std::placeholders::_1, std::chrono::seconds(5));
  }

  run_test(argc, argv, create_pub_sub_system, run_func);
}
