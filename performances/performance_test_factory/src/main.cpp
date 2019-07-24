/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <sys/wait.h>


#include "rclcpp/rclcpp.hpp"

#include "performance_test/ros2/node.hpp"
#include "performance_test_factory/factory.hpp"

#include "cli/options.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<performance_test::Node>("node_name");

    // Load topology from json file
    performance_test::TemplateFactory factory = performance_test::TemplateFactory(true);

    factory.add_subscriber_from_strings(node, "10b", "my_topic", performance_test::Tracker::TrackingOptions());
}
