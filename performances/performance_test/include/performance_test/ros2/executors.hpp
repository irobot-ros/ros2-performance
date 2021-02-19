/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace performance_test {

struct NamedExecutor
{
    std::shared_ptr<rclcpp::Executor> executor;
    std::string name;
};

enum ExecutorType
{
    SINGLE_THREADED_EXECUTOR = 1,
    STATIC_SINGLE_THREADED_EXECUTOR = 2,
};

enum NodeType
{
    RCLCPP_NODE = 1,
    RCLCPP_LIFECYCLE_NODE = 2,
};

std::ostream &operator<<(std::ostream &os, const ExecutorType &t) {
    std::string executor_name;
    switch (t) {
        case SINGLE_THREADED_EXECUTOR:
            executor_name = "SingleThreadedExecutor";
            break;
        case STATIC_SINGLE_THREADED_EXECUTOR:
            executor_name = "StaticSingleThreadedExecutor";
            break;
        default:
            executor_name = "Unknown ExecutorType";
            break;
    }

    return os << executor_name;
}

std::ostream &operator<<(std::ostream &os, const NodeType &t) {
    std::string node_type;
    switch (t) {
        case RCLCPP_NODE:
            node_type = "Node";
            break;
        case RCLCPP_LIFECYCLE_NODE:
            node_type = "LifecycleNode";
            break;
        default:
            node_type = "Unknown NodeType";
            break;
    }

    return os << node_type;
}

}
