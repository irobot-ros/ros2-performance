/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/resource_usage_logger.hpp"

#include "cxxopts.hpp"

int main(int argc, char ** argv)
{
    /**
     * Configuration
     */

    // experiment default values
    int n_clients = 2;
    int n_services = 1;
    std::string msg_type = "10b";
    float frequency = 10;
    int executors = 0;
    int use_ipc = 0;
    std::string ros_namespace = "";
    int experiment_duration = 5;
    int monitor_stats = 0;
    std::string experiment_name = "";
    std::string experiment_path = ".";
    int verbose = 0;

    std::string events_file_path = "";
    std::string ru_file_path = "";
    std::string latency_file_path = "";

    // parse the command line arguments and eventually overwrite the default values
    cxxopts::Options options("client_nodes_main", "Run some ROS2 client nodes");
    options.add_options()
    ("c,clients", "Number of client nodes",
        cxxopts::value<int>(n_clients)->default_value(std::to_string(n_clients)))
    ("s,services", "Number of service ndoes",
        cxxopts::value<int>(n_services)->default_value(std::to_string(n_services)))
    ("msg_type", "Type of service",
        cxxopts::value<std::string>(msg_type)->default_value(msg_type))
    ("f,frequency", "Request frequency",
        cxxopts::value<float>(frequency)->default_value(std::to_string(frequency)))
    ("executors", "Threading model: 0 each node has its own thread, 1 all nodes are in the same executor",
        cxxopts::value<int>(executors)->default_value(std::to_string(executors)))
    ("use_ipc", "Activate IntraProcessCommunication (0 or 1 accepted arguments)",
        cxxopts::value<int>(use_ipc)->default_value(std::to_string(use_ipc)))
    ("ros_namespace", "Create every node under this namespace",
        cxxopts::value<std::string>(ros_namespace)->default_value(ros_namespace))
    ("t, duration", "Duration in seconds",
        cxxopts::value<int>(experiment_duration)->default_value(std::to_string(experiment_duration)))
    ("monitor_stats", "Monitor CPU, RAM and events and print them to file (0 or 1 accepted arguments)",
        cxxopts::value<int>(monitor_stats)->default_value(std::to_string(monitor_stats)))
    ("experiment_name", "Experiment name",
        cxxopts::value<std::string>(experiment_name)->default_value(experiment_name))
    ("experiment_path", "Experiment path",
        cxxopts::value<std::string>(experiment_path)->default_value(experiment_path))
    ("verbose", "Print runtime debug information (0 or 1 accepted arguments)",
        cxxopts::value<int>(verbose)->default_value(std::to_string(verbose)))
    ;
    options.parse(argc, argv);

    if (!experiment_name.empty()){
        latency_file_path = experiment_path + "/lat_rel_" + experiment_name + ".csv";
        if (monitor_stats){
            events_file_path = experiment_path + "/events_" + experiment_name + ".csv";
            ru_file_path = experiment_path + "/cpu_ram_" + experiment_name + ".csv";
        }
    }

    performance_test::ResourceUsageLogger ru_logger(ru_file_path);
    size_t m_size = performance_test::TemplateFactory::get_msg_size(msg_type);
    ru_logger.set_system_info(n_services, n_clients, frequency, m_size);
    // Start resources logger
    if (monitor_stats){
        ru_logger.start();
    }

    /**
     * Execution
     */

    std::cout<<"Start test"<<std::endl;

    rclcpp::init(argc, argv);

    performance_test::TemplateFactory factory(use_ipc, verbose, ros_namespace);
    performance_test::System ros2_system(executors);
    ros2_system.enable_events_logger(events_file_path);

    std::vector<std::shared_ptr<performance_test::Node>> server_nodes =
        factory.create_server_nodes(
            n_clients,
            n_clients + n_services,
            msg_type,
            rmw_qos_profile_default);

    ros2_system.add_node(server_nodes);

    std::cout<<"Servers created!"<<std::endl;

    // allow some time for all the nodes to be created/discovered
    std::this_thread::sleep_for(std::chrono::seconds(1));

    bool wait_discovery = false;
    ros2_system.spin(experiment_duration, wait_discovery);

    rclcpp::shutdown();

    std::this_thread::sleep_for(500ms);

    std::cout << "End test" << std::endl << std::endl;

    /**
     * Visualization
     */

    if (monitor_stats){
        ru_logger.stop();
    }

    ros2_system.print_latency_all_stats();

    return 0;
}

