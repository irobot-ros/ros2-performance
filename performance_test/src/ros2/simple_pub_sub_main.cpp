#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/options.hpp"
#include "performance_test/ros2/print_utilities.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // experiment default values
    int n_subscribers = 2;
    int n_publishers = 1;
    std::string msg_type = "10b";
    int msg_size=0;
    float frequency = 10;
    int executors = 0;
    int experiment_duration = 5;
    std::string filename = "";
    std::string ros_namespace = "";
    bool verbose = false;

    bool ret = parse_command_options(
        argc, argv,
        &n_publishers,          // n_publishers
        &n_subscribers,         // n_subscribers
        nullptr,                // n_clients
        nullptr,                // n_services
        &msg_type,              // msg_type
        &msg_size,              // msg_size
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

    //TODO: do this better
    // if there are only publishers or only subscribers, the command line argument frequency applies to them.
    // elsewhere it applies to publishers.
    float subscribers_spin_frequency = -1;
    float publisher_spin_frequency = frequency;
    if (n_publishers == 0){
        subscribers_spin_frequency = frequency;
    }

    TemplateFactory ros2system(ros_namespace);

    std::cout<<"Start test"<<std::endl;

    std::vector<std::shared_ptr<MultiNode>> pub_nodes = ros2system.create_publishers(n_subscribers, n_subscribers + n_publishers, msg_type, verbose, custom_qos_profile);

    std::cout<<"Publishers created!"<<std::endl;

    std::vector<std::shared_ptr<MultiNode>> sub_nodes = ros2system.create_subscribers(0, n_subscribers, n_publishers, msg_type, verbose, custom_qos_profile);

    std::cout<<"Subscribers created!"<<std::endl;

    // allow some time for all the nodes to be created/discovered
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ros2system.start_subscribers(sub_nodes, subscribers_spin_frequency, executor);

    ros2system.start_publishers(pub_nodes, publisher_spin_frequency, experiment_duration, msg_size, executor);

    if (executor != nullptr){
        std::thread thread([executor](){
            executor->spin();
        });
        thread.detach();
    }

    // the publisher will send messages for "experiment_duration" seconds,
    // but we sleep slightly more to ensure that all messages are delivered
    std::this_thread::sleep_for(std::chrono::seconds(experiment_duration + 1));

    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;

    // store pointer to nodes into a unique vector
    std::vector<std::shared_ptr<MultiNode>> nodes_vector;
    nodes_vector.insert(nodes_vector.end(), sub_nodes.begin(), sub_nodes.end());
    nodes_vector.insert(nodes_vector.end(), pub_nodes.begin(), pub_nodes.end());

    print_node_stats(filename, nodes_vector, msg_type, experiment_duration);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout<<"End test"<<std::endl;

    return 0;
}


