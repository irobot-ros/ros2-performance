#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/composable_publisher.hpp>
#include <composition_benchmark/composable_subscriber.hpp>
#include <composition_benchmark/helpers/helper_options.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>
#include <performance_test/utils/node_options.hpp>

static
std::vector<IRobotNodePtr> create_pub_sub_system(int argc, char ** argv)
{
  auto options = CompositionOptions(argc, argv);

  std::vector<IRobotNodePtr> nodes;

  std::vector<rclcpp::Parameter> pub_parameters = {
    {"topic", "dummy_topic"},
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
      {"topic", "dummy_topic"}
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

  run_func_t run_func;
  if (*options.spin_type == "spin") {
    run_func = std::bind(spin_task, std::placeholders::_1, MAX_HOURS);
  } else if (*options.spin_type == "spin_future") {
    run_func = std::bind(spin_future_complete_task, std::placeholders::_1, MAX_HOURS);
  } else if (*options.spin_type == "spin_isolated") {
    run_func = std::bind(spin_isolated_task, std::placeholders::_1, MAX_HOURS);
  } else if (*options.spin_type == "spin_some") {
    run_func = std::bind(spin_some_task, std::placeholders::_1, MAX_HOURS);
  }

  run_test(
    argc,
    argv,
    create_pub_sub_system,
    run_func);
}
