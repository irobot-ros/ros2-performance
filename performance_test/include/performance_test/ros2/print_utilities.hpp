#ifndef __PRINT_UTILITIES_HPP__
#define __PRINT_UTILITIES_HPP__

#include <string>
#include <vector>
#include "performance_test/ros2/multi_node.hpp"


void print_node_stats(std::string filename, std::vector<std::shared_ptr<MultiNode>> nodes, std::string msg_type, int experiment_duration);


#endif  // __PRINT_UTILITIES_HPP__