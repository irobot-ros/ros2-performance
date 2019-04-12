#include <algorithm>
#include <map>

#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/templated_multi_node.hpp"
#include "performance_test/ros2/names_utilities.hpp"

#include "performance_test/msg/stamped10b.hpp"
#include "performance_test/msg/stamped100b.hpp"
#include "performance_test/msg/stamped250b.hpp"
#include "performance_test/msg/stamped1kb.hpp"
#include "performance_test/msg/stamped10kb.hpp"
#include "performance_test/msg/stamped100kb.hpp"
#include "performance_test/msg/stamped250kb.hpp"
#include "performance_test/msg/stamped1mb.hpp"
#include "performance_test/msg/stamped4mb.hpp"
#include "performance_test/msg/stamped8mb.hpp"
#include "performance_test/msg/stamped_vector.hpp"

using namespace std::chrono_literals;


std::shared_ptr<MultiNode> TemplateFactory::make_templated(const std::string& msg_type, int id)
{

    std::string node_name = id_to_node_name(id);

    return make_templated(msg_type, node_name);

}


std::shared_ptr<MultiNode> TemplateFactory::make_templated(const std::string& msg_type, std::string name)
{

    // transform input string to all lowercase
    std::string lowercase_string(msg_type);
    std::transform(lowercase_string.begin(), lowercase_string.end(), lowercase_string.begin(), ::tolower);

    using functionType = std::function<std::shared_ptr<MultiNode>()>;

    std::map<std::string, functionType> factoryMap{
        {"10b",    [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped10b>>(name, _ros2_namespace); } },
        {"100b",   [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped100b>>(name, _ros2_namespace); } },
        {"250b",   [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped250b>>(name, _ros2_namespace); } },
        {"1kb",    [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped1kb>>(name, _ros2_namespace); } },
        {"10kb",   [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped10kb>>(name, _ros2_namespace); } },
        {"100kb",  [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped100kb>>(name, _ros2_namespace); } },
        {"250kb",  [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped250kb>>(name, _ros2_namespace); } },
        {"1mb",    [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped1mb>>(name, _ros2_namespace); } },
        {"4mb",    [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped4mb>>(name, _ros2_namespace); } },
        {"8mb",    [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Stamped8mb>>(name, _ros2_namespace); } },
        {"vector", [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::StampedVector>>(name, _ros2_namespace); } }
    };

    if (factoryMap.find(lowercase_string) == factoryMap.end()){
        assert(0 && "Error: provided unknown msgType to make_templated");
    }

    // creates an instance of TemplatedMultiNode with the provided msg_type and name.
    // returns a shared pointer to its base class
    return factoryMap[lowercase_string]();

}


std::vector<std::shared_ptr<MultiNode>> TemplateFactory::create_subscribers(
    int start_id,
    int end_id,
    int n_publishers,
    std::string msg_type,
    bool verbose,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<MultiNode>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id ++){

        auto node = make_templated(msg_type, node_id);

        if (verbose){
            auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) { assert(0 && "Error setting logger verbosity"); }
        }

        for (int k = 0; k < n_publishers; k ++){

            int subscriber_id = k + end_id;

            node->add_subscriber(subscriber_id, custom_qos_profile);
        }

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}


std::vector<std::shared_ptr<MultiNode>> TemplateFactory::create_publishers(
    int start_id,
    int end_id,
    std::string msg_type,
    bool verbose,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<MultiNode>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++){

        auto node = make_templated(msg_type, node_id);

        if (verbose){
            auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) { assert(0 && "Error setting logger verbosity"); }
        }

        int publisher_id = node_id;

        node->add_publisher(publisher_id, custom_qos_profile);
        nodes_vector.push_back(node);
    }

    return nodes_vector;

}


std::vector<std::shared_ptr<MultiNode>> TemplateFactory::create_clients(
    int start_id,
    int end_id,
    int n_services,
    std::string msg_type,
    bool verbose,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<MultiNode>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++){

        auto node = make_templated(msg_type, node_id);

        if (verbose){
            auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) { assert(0 && "Error setting logger verbosity"); }
        }

        for (int k = 0; k < n_services; k ++){

            int client_id = k + end_id;

            node->add_client(client_id, custom_qos_profile);
        }

        nodes_vector.push_back(node);
    }

    return nodes_vector;

}


std::vector<std::shared_ptr<MultiNode>> TemplateFactory::create_servers(
    int start_id,
    int end_id,
    std::string msg_type,
    bool verbose,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<MultiNode>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++){

        auto node = make_templated(msg_type, node_id);

        if (verbose){
            auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) { assert(0 && "Error setting logger verbosity"); }
        }

        int service_id = node_id;

        node->add_service(service_id,custom_qos_profile);

        nodes_vector.push_back(node);
    }

    return nodes_vector;

}


void TemplateFactory::start_subscribers(
    std::vector<std::shared_ptr<MultiNode>> nodes,
    float frequency,
    rclcpp::executor::Executor::SharedPtr executor)
{
    for (auto node : nodes){

        if (executor != nullptr){
            executor->add_node(node);
        }
        else {
            std::thread thread = std::thread(&MultiNode::simple_spin_task, node, frequency);
            thread.detach();
        }
    }
}


void TemplateFactory::start_publishers(
    std::vector<std::shared_ptr<MultiNode>> nodes,
    float frequency,
    int task_duration_sec,
    int msg_size,
    rclcpp::executor::Executor::SharedPtr executor)
{
    for (auto node : nodes){

        if (executor != nullptr){
            node->add_timer(frequency, std::bind(&MultiNode::publish_messages, node, frequency, task_duration_sec, msg_size));
            executor->add_node(node);
        }
        else{
            std::thread thread = std::thread(&MultiNode::simple_publisher_task, node, frequency, task_duration_sec, msg_size);
            thread.detach();
        }
    }
}


void TemplateFactory::start_clients(
    std::vector<std::shared_ptr<MultiNode>> nodes,
    float frequency,
    int task_duration_sec,
    rclcpp::executor::Executor::SharedPtr executor)
{
    for (auto node : nodes){

        if (executor != nullptr){
            node->add_timer(frequency, std::bind(&MultiNode::send_requests, node, frequency, task_duration_sec));
            executor->add_node(node);
        }
        else {
            std::thread thread = std::thread(&MultiNode::simple_client_task, node, frequency, task_duration_sec);
            thread.detach();
        }
    }
}


void TemplateFactory::start_servers(
    std::vector<std::shared_ptr<MultiNode>> nodes,
    float frequency,
    rclcpp::executor::Executor::SharedPtr executor)
{
    for (auto node : nodes){
        if (executor != nullptr){
            executor->add_node(node);
        }
        else {
            std::thread thread = std::thread(&MultiNode::simple_spin_task, node, frequency);
            thread.detach();
        }
    }
}
