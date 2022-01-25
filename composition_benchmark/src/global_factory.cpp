#include <atomic>
#include <iostream>
#include <mutex>

#include <composition_benchmark/global_factory.hpp>

namespace global_factory
{

static size_t s_current_id {0};
static std::vector<NodeArguments> s_nodes_args {};
static std::mutex s_mutex {};

static NodeArguments get_node_args()
{
  std::lock_guard<std::mutex> lock(s_mutex);
  if (s_current_id < s_nodes_args.size()) {
    return s_nodes_args[s_current_id];
  } else {
    std::cerr<< "Exceeding available node args " << s_nodes_args.size() << " with "<< s_current_id << std::endl;
    return NodeArguments();
  }
}

void setup_nodes(const std::vector<NodeArguments>& nodes_args)
{
  std::lock_guard<std::mutex> lock(s_mutex);
  if (!s_nodes_args.empty()) {
    std::cerr<< "Setting up nodes will erase existing information about " << s_nodes_args.size() << " nodes"<< std::endl;
  }
  s_nodes_args = nodes_args;
}

void mark_node_created()
{
  std::lock_guard<std::mutex> lock(s_mutex);
  s_current_id++;
}

std::string get_node_name()
{
  return get_node_args().name;
}
std::string get_namespace()
{
  return get_node_args().ros_namespace;
}
size_t get_executor_id()
{
  return get_node_args().executor_id;
}

}
