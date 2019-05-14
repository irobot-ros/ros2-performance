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
#include <string>

#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/resource_usage_logger.hpp"
#include "performance_test/ros2/tracker.hpp"

#include "cxxopts.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{

    std::string this_file_path = __FILE__;
    std::string this_dir_path = this_file_path.substr(0, this_file_path.rfind("/"));
    std::string json_path = this_dir_path + std::string("/simple_architecture.json");
    int use_ipc = 1;
    int experiment_duration = 5;
    int resources_sampling_per_ms = 500;
    performance_test::Tracker::TrackingOptions tracking_options;
    std::string experiment_path = ".";


    // parse the command line arguments and eventually overwrite the default values
    cxxopts::Options options("json_system_main", "Create a ROS2 system at runtime as defined by a JSON file");
    options.add_options()
    ("j, json", "path to the json file to load",
        cxxopts::value<std::string>(json_path)->default_value(json_path))
    ("use_ipc", "Activate IntraProcessCommunication (0 or 1 accepted arguments)",
        cxxopts::value<int>(use_ipc)->default_value(std::to_string(use_ipc)))
    ("t, duration", "Duration in seconds",
        cxxopts::value<int>(experiment_duration)->default_value(std::to_string(experiment_duration)))
    ("s, sampling", "resources sampling period",
        cxxopts::value<int>(resources_sampling_per_ms)->default_value(std::to_string(resources_sampling_per_ms)),"msec")
    ("late-percentage", "a msg with greater latency than this percentage of the msg publishing period is considered late",
        cxxopts::value<int>(tracking_options.late_percentage)->default_value(std::to_string(tracking_options.late_percentage)),"%")
    ("late-absolute", "a msg with greater latency than this is considered late",
        cxxopts::value<int>(tracking_options.late_absolute_us)->default_value(std::to_string(tracking_options.late_absolute_us)),"usec")
    ("too-late-percentage", "a msg with greater latency than this percentage of the msg publishing period is considered lost",
        cxxopts::value<int>(tracking_options.too_late_percentage)->default_value(std::to_string(tracking_options.too_late_percentage)),"%")
    ("too-late-absolute", "a msg with greater latency than this is considered lost",
        cxxopts::value<int>(tracking_options.too_late_absolute_us)->default_value(std::to_string(tracking_options.too_late_absolute_us)),"usec")
    ("experiment_path", "Experiment path",
        cxxopts::value<std::string>(experiment_path)->default_value(experiment_path))
    ;
    options.parse(argc, argv);

    std::cout << "Json file to load: "<< json_path<<std::endl;
    std::cout << "Intra-process-communication: " << (use_ipc ? "on" : "off") << std::endl;
    std::cout << "Run test for: " << experiment_duration << " seconds" << std::endl;
    std::cout << "Sampling resources every " << resources_sampling_per_ms << "ms" << std::endl;
    std::cout << "Start test" << std::endl;


    std::string create_output_dir_command = std::string("mkdir -p ") + experiment_path + std::string("/log");
    system(create_output_dir_command.c_str());
    std::string resources_output_path = experiment_path + std::string("/log/resources.txt");
    std::string events_output_path = experiment_path + std::string("/log/events.txt");
    std::string latency_all_output_path = experiment_path + std::string("/log/latency_all.txt");
    std::string latency_total_output_path = experiment_path + std::string("/log/latency_total.txt");

    // Start resources logger
    performance_test::ResourceUsageLogger ru_logger(resources_output_path);
    ru_logger.start(std::chrono::milliseconds(resources_sampling_per_ms));

    rclcpp::init(argc, argv);

    // Architecture
    int executors = 0; // set to 1 if you want to add all nodes to the same executor
    performance_test::System ros2_system(executors);
    ros2_system.enable_events_logger(events_output_path);

    performance_test::TemplateFactory factory(use_ipc);

    auto nodes_vec = factory.parse_topology_from_json(json_path);

    ros2_system.add_node(nodes_vec);

    ros2_system.spin(experiment_duration);

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
