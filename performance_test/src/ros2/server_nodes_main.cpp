#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/options.hpp"
#include "performance_test/ros2/print_utilities.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // experiment default values
    int n_clients = 2;
    int n_services = 1;
    std::string msg_type = "10b";
    float frequency = -1;
    int executors = 0;
    int experiment_duration = 5;
    std::string filename = "";
    std::string ros_namespace = "";
    bool verbose = false;


    bool ret = parse_command_options(
        argc, argv,
        nullptr,                // n_publishers
        nullptr,                // n_subscribers
        &n_clients,             // n_clients
        &n_services,            // n_services
        &msg_type,              // msg_type
        nullptr,                // msg_size
        &executors,             // executors
        &frequency,             // frequency
        &experiment_duration,   // experiment_duration
        &filename,              // filename
        &ros_namespace          // namespace
    );

    if (!ret){
        rclcpp::shutdown();
        return 1;
    }

    // qos profile used for this experiment
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.depth = 10;
    custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    custom_qos_profile.avoid_ros_namespace_conventions = false;

    rclcpp::executor::Executor::SharedPtr executor = nullptr;
    if (executors == 1){
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }

    TemplateFactory ros2system(ros_namespace);

    std::cout<<"Start test"<<std::endl;

    std::vector<std::shared_ptr<MultiNode>> server_nodes = ros2system.create_servers(n_clients, n_clients + n_services, msg_type, verbose, custom_qos_profile);

    std::cout<<"Servers created!"<<std::endl;

    // allow some time for all the nodes to be created/discovered
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ros2system.start_servers(server_nodes, frequency, executor);

    if (executor != nullptr){
        std::thread thread([executor](){
            executor->spin();
        });
        thread.detach();
    }

    // the clients will send requests for "experiment_duration" seconds,
    // but we sleep slightly more to ensure that all requests are processed
    std::this_thread::sleep_for(std::chrono::seconds(experiment_duration + 1));

    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;

    print_node_stats(filename, server_nodes, msg_type, experiment_duration);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout<<"End test"<<std::endl;

    return 0;
}

