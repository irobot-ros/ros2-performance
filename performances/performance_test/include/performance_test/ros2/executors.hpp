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
    EVENTS_EXECUTOR = 2,
    STATIC_SINGLE_THREADED_EXECUTOR,
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
        case EVENTS_EXECUTOR:
            executor_name = "EventsExecutor";
            break;
        default:
            executor_name = "Unknown ExecutorType";
            break;
    }

    return os << executor_name;
}

}
