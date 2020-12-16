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

#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/communication.hpp"
#include "performance_test/ros2/resource_usage_logger.hpp"

#include "performance_test_factory/factory.hpp"

#include "cli/options.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    std::string make_dir = "mkdir -p mauro";
    const auto ret = system(make_dir.c_str());
    static_cast<void>(ret);
    std::string dir_name     = "mauro";
    std::string resources_output_path     = dir_name + "/resources.txt";
    std::string events_output_path        = dir_name + "/events.txt";
    std::string latency_all_output_path   = dir_name + "/latency_all.txt";
    std::string latency_total_output_path = dir_name + "/latency_total.txt";

    // Start resources logger
    performance_test::ResourceUsageLogger ru_logger(resources_output_path);
    ru_logger.start(std::chrono::milliseconds(1000));
    std::cout << "\n\n0. Init" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));


    std::cout << "\n\n1. rclcpp::remove_ros_arguments" << std::endl;
    auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "\n\n1.5 parse options" << std::endl;


    std::vector<char *> non_ros_args_c_strings;
    for (auto & arg : non_ros_args) {
        non_ros_args_c_strings.push_back(&arg.front());
    }
    int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());
    auto options = irobot_benchmark::Options(non_ros_argc, non_ros_args_c_strings.data());

    auto json_list = options.topology_json_list;

    std::cout << "Topology file(s): " << std::endl;
    for(const auto& json : json_list) std::cout << json << std::endl;

    // Get the system executor from options

    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "\n\n2. performance_test::systemExecutor" << std::endl;
    performance_test::systemExecutor system_executor;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    system_executor = static_cast<performance_test::systemExecutor>(options.executor);

    switch (system_executor)
    {
        using namespace performance_test;

        case EVENTS_EXECUTOR:
            std::cout << "System executor: EventsExecutor" << std::endl;
            break;

        case SINGLE_THREADED_EXECUTOR:
            std::cout << "System executor: SingleThreadedExecutor" << std::endl;
            break;

        case STATIC_SINGLE_THREADED_EXECUTOR:
        default:
            std::cout << "System executor: StaticSingleThreadedExecutor" << std::endl;
            break;
    }

    std::cout << "Intra-process-communication: " << (options.ipc ? "on" : "off") << std::endl;
    std::cout << "Parameter services: " << (options.ros_params ? "on" : "off") << std::endl;
    std::cout << "Naming threads: " << (options.name_threads ? "on" : "off") << std::endl;
    std::cout << "Run test for: " << options.duration_sec << " seconds" << std::endl;
    std::cout << "Sampling resources every " << options.resources_sampling_per_ms << "ms" << std::endl;
    std::cout << "Logging events statistics: " << (options.tracking_options.is_enabled ? "on" : "off") << std::endl;
    std::cout << "Start test" << std::endl;

    std::string topology_json;

    pid_t pid = getpid();

    for (auto json = json_list.begin(); json != json_list.end(); json++)
    {
        topology_json = *json;

        // Fork only the for the first (n-1) topologies
        if (json != json_list.end() - 1)
        {
            pid = fork();

            // If is a child process, break
            if (pid == 0)
            {
                 break;
            }
        }
    }

    // Create results dir based on the topology name
    // const size_t last_slash = topology_json.find_last_of("/");
    // std::string topology_basename = topology_json.substr(last_slash + 1, topology_json.length());
    // std::string dir_name = topology_basename.substr(0,topology_basename.length()-5) + "_log";



    rclcpp::init(argc, argv);

    performance_test::System ros2_system(system_executor);

    if (options.tracking_options.is_enabled) {
        ros2_system.enable_events_logger(events_output_path);
    }

    // Load topology from json file
    auto factory = performance_test::TemplateFactory(options.ipc, options.ros_params);

    auto nodes_vec = factory.parse_topology_from_json(topology_json, options.tracking_options);
    ros2_system.add_node(nodes_vec);

    // now the system is complete and we can make it spin for the requested duration
    bool wait_for_discovery = true;

    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "\n\n5. start test" << std::endl;

    ros2_system.spin(options.duration_sec, wait_for_discovery, options.name_threads);

    // terminate the experiment
    ru_logger.stop();
    rclcpp::shutdown();
    std::this_thread::sleep_for(500ms);

    ros2_system.print_latency_all_stats();

    if (json_list.size() > 1)
    {
        std::cout << std::endl << "Process total:" << std::endl;
        ros2_system.print_latency_total_stats();
    }

    std::cout << std::endl;
    ros2_system.save_latency_all_stats(latency_all_output_path);
    ros2_system.save_latency_total_stats(latency_total_output_path);

    // Parent process: wait for children to exit and print system stats
    if (pid != 0)
    {
        if (json_list.size() > 1)
        {
            waitpid(getpid()+1, &pid, 0);
        }
        std::cout << "System total:" << std::endl;
        ros2_system.print_agregate_stats(json_list);
    }
}
