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

#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/communication.hpp"
#include "performance_test/ros2/resource_usage_logger.hpp"
#include "performance_test/ros2/template_factory.hpp"

#include "options.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    auto options = benchmark::Options(argc, argv);

    std::cout << "Topology file: " << options.topology_json_path << std::endl;
    std::cout << "Intra-process-communication: " << (options.ipc ? "on" : "off") << std::endl;
    std::cout << "Run test for: " << options.duration_sec << " seconds" << std::endl;
    std::cout << "Sampling resources every " << options.resources_sampling_per_ms << "ms" << std::endl;
    std::cout << "Start test" << std::endl;

    system("mkdir -p log");
    std::string resources_output_path = "log/resources.txt";
    std::string events_output_path = "log/events.txt";
    std::string latency_all_output_path = "log/latency_all.txt";
    std::string latency_total_output_path = "log/latency_total.txt";

    // Start resources logger
    performance_test::ResourceUsageLogger ru_logger(resources_output_path);
    ru_logger.start(std::chrono::milliseconds(options.resources_sampling_per_ms));

    rclcpp::init(argc, argv);

    int executors = 0; // set to 1 if you want to add all nodes to the same executor
    performance_test::System ros2_system(executors);
    ros2_system.enable_events_logger(events_output_path);

    // Load topology from json file
    performance_test::TemplateFactory factory = performance_test::TemplateFactory(options.ipc);

    auto nodes_vec = factory.parse_topology_from_json(options.topology_json_path);

    // Check if file contained a valid topology
    if(nodes_vec.empty()) {
        std::cout << "Error reading topology file." << std::endl;
        exit(1);
    }

    ros2_system.add_node(nodes_vec);

    // now the system is complete and we can make it spin for the requested duration
    ros2_system.spin(options.duration_sec);

    // terminate the experiment
    ru_logger.stop();
    rclcpp::shutdown();
    std::this_thread::sleep_for(500ms);

    ros2_system.print_latency_all_stats();
    std::cout << std::endl;
    std::cout << "System total:" << std::endl;
    ros2_system.print_latency_total_stats();
    ros2_system.save_latency_all_stats(latency_all_output_path);
    ros2_system.save_latency_total_stats(latency_total_output_path);

    std::cout << std::endl;


}
