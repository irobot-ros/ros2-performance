
#include <rclcpp/rclcpp.hpp>
#include <performance_test/utils/cli_args.hpp>

namespace performance_test
{

std::vector<char *> get_non_ros_args(int argc, char** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }

  return non_ros_args_c_strings;
}

}
