
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace global_factory
{

struct NodeArguments
{
  std::string name {"node"};
  std::string ros_namespace {""};
  size_t executor_id {0};
};

void setup_factory(const std::vector<NodeArguments>& nodes_args);

size_t get_num_registered_nodes();

void mark_node_created();

std::string get_node_name();

std::string get_namespace();

size_t get_executor_id();

}
