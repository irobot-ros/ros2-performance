/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <algorithm>
#include <map>

#include "performance_test_msgs/msg/stamped10b.hpp"
#include "performance_test_msgs/msg/stamped100b.hpp"
#include "performance_test_msgs/msg/stamped250b.hpp"
#include "performance_test_msgs/msg/stamped1kb.hpp"
#include "performance_test_msgs/msg/stamped10kb.hpp"
#include "performance_test_msgs/msg/stamped100kb.hpp"
#include "performance_test_msgs/msg/stamped250kb.hpp"
#include "performance_test_msgs/msg/stamped1mb.hpp"
#include "performance_test_msgs/msg/stamped4mb.hpp"
#include "performance_test_msgs/msg/stamped8mb.hpp"
#include "performance_test_msgs/msg/stamped_vector.hpp"

#include "performance_test_msgs/srv/stamped10b.hpp"

#include "benchmark_msgs/msg/stamped3_float32.hpp"
#include "benchmark_msgs/msg/stamped4_float32.hpp"
#include "benchmark_msgs/msg/stamped4_int32.hpp"
#include "benchmark_msgs/msg/stamped9_float32.hpp"
#include "benchmark_msgs/msg/stamped12_float32.hpp"
#include "benchmark_msgs/msg/stamped_int64.hpp"

#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/names_utilities.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;


std::shared_ptr<performance_test::Node> performance_test::TemplateFactory::create_node(
    std::string name,
    bool use_ipc,
    bool verbose,
    std::string ros2_namespace)
{

    auto node = std::make_shared<performance_test::Node>(name, ros2_namespace, use_ipc);

    if (verbose){
        auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) { assert(0 && "Error setting logger verbosity"); }
    }

    return node;
}


std::vector<std::shared_ptr<performance_test::Node>> performance_test::TemplateFactory::create_subscriber_nodes(
    int start_id,
    int end_id,
    int n_publishers,
    std::string msg_type,
    msg_pass_by_t msg_pass_by,
    Tracker::TrackingOptions tracking_options,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id ++){

        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _verbose_mode, _ros2_namespace);

        // TODO: pass publisher list instead of n_publishers, to select the IDs (
        // default is a list from 0 to n_pubs or directly from n_subs to n_pubs)
        for (int k = 0; k < n_publishers; k ++){

            int topic_id = k + end_id;
            std::string topic_name = id_to_topic_name(topic_id);

            this->add_subscriber_from_strings(node, msg_type, topic_name, tracking_options, msg_pass_by, custom_qos_profile);
        }

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}


std::vector<std::shared_ptr<performance_test::Node>> performance_test::TemplateFactory::create_periodic_publisher_nodes(
    int start_id,
    int end_id,
    float frequency,
    std::string msg_type,
    msg_pass_by_t msg_pass_by,
    size_t msg_size,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++){

        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _verbose_mode, _ros2_namespace);

        int topic_id = node_id;
        std::string topic_name = id_to_topic_name(topic_id);

        int period = (1000/frequency);
        std::chrono::milliseconds period_ms = std::chrono::milliseconds(period);

        this->add_periodic_publisher_from_strings(node, msg_type, topic_name, msg_pass_by, custom_qos_profile, period_ms, msg_size);

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}


std::vector<std::shared_ptr<performance_test::Node>> performance_test::TemplateFactory::create_periodic_client_nodes(
    int start_id,
    int end_id,
    int n_services,
    float frequency,
    std::string srv_type,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++){

        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _verbose_mode, _ros2_namespace);

        int period = (1000/frequency);
        std::chrono::milliseconds period_ms = std::chrono::milliseconds(period);

        for (int k = 0; k < n_services; k ++){

            int service_id = k + end_id;
            std::string service_name = id_to_service_name(service_id);

            this->add_periodic_client_from_strings(node, srv_type, service_name, custom_qos_profile, period_ms);

        }

        nodes_vector.push_back(node);
    }

    return nodes_vector;

}


std::vector<std::shared_ptr<performance_test::Node>> performance_test::TemplateFactory::create_server_nodes(
    int start_id,
    int end_id,
    std::string srv_type,
    rmw_qos_profile_t custom_qos_profile)
{
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++){

        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _verbose_mode, _ros2_namespace);

        int service_id = node_id;
        std::string service_name = id_to_service_name(service_id);

        this->add_server_from_strings(node, srv_type, service_name, custom_qos_profile);

        nodes_vector.push_back(node);
    }

    return nodes_vector;

}


void performance_test::TemplateFactory::add_subscriber_from_strings(
    std::shared_ptr<performance_test::Node> n,
    std::string msg_type,
    std::string topic_name,
    Tracker::TrackingOptions tracking_options,
    msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t custom_qos_profile)
{

    const std::map<std::string, std::function<void()>>  subscribers_factory{
        {"10b",         [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped10b>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"100b",        [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped100b>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"250b",        [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped250b>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"1kb",         [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped1kb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"10kb",        [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped10kb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"100kb",       [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped100kb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"250kb",       [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped250kb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"1mb",         [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped1mb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"4mb",         [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped4mb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"8mb",         [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped8mb>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"3float32",    [&] { n->add_subscriber(performance_test::Topic<benchmark_msgs::msg::Stamped3Float32>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"4float32",    [&] { n->add_subscriber(performance_test::Topic<benchmark_msgs::msg::Stamped4Float32>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"4int32",      [&] { n->add_subscriber(performance_test::Topic<benchmark_msgs::msg::Stamped4Int32>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"9float32",    [&] { n->add_subscriber(performance_test::Topic<benchmark_msgs::msg::Stamped9Float32>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"12float32",   [&] { n->add_subscriber(performance_test::Topic<benchmark_msgs::msg::Stamped12Float32>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"int64",       [&] { n->add_subscriber(performance_test::Topic<benchmark_msgs::msg::StampedInt64>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
        {"vector",      [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::StampedVector>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } }
    };

    if (subscribers_factory.find(msg_type) == subscribers_factory.end()){
        assert(0 && "unknown msg type passed to factory method!" );
    }

    subscribers_factory.at(msg_type)();

}


void performance_test::TemplateFactory::add_periodic_publisher_from_strings(
    std::shared_ptr<performance_test::Node> n,
    std::string msg_type,
    std::string topic_name,
    msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t custom_qos_profile,
    std::chrono::milliseconds period_ms,
    size_t msg_size)
{

    const std::map<std::string, std::function<void()>>  publishers_factory{
        {"10b",         [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped10b>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"100b",        [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped100b>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"250b",        [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped250b>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"1kb",         [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped1kb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"10kb",        [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped10kb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"100kb",       [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped100kb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"250kb",       [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped250kb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"1mb",         [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped1mb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"4mb",         [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped4mb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"8mb",         [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::Stamped8mb>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"3float32",    [&] { n->add_periodic_publisher(performance_test::Topic<benchmark_msgs::msg::Stamped3Float32>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"4float32",    [&] { n->add_periodic_publisher(performance_test::Topic<benchmark_msgs::msg::Stamped4Float32>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"4int32",      [&] { n->add_periodic_publisher(performance_test::Topic<benchmark_msgs::msg::Stamped4Int32>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"9float32",    [&] { n->add_periodic_publisher(performance_test::Topic<benchmark_msgs::msg::Stamped9Float32>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"12float32",   [&] { n->add_periodic_publisher(performance_test::Topic<benchmark_msgs::msg::Stamped12Float32>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"int64",       [&] { n->add_periodic_publisher(performance_test::Topic<benchmark_msgs::msg::StampedInt64>(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },
        {"vector",      [&] { n->add_periodic_publisher(performance_test::Topic<performance_test_msgs::msg::StampedVector>(topic_name), period_ms,  msg_pass_by, custom_qos_profile, msg_size); } }
    };

    if (publishers_factory.find(msg_type) == publishers_factory.end()){
        assert(0 && "unknown msg type passed to factory method!" );
    }

    publishers_factory.at(msg_type)();
}


void performance_test::TemplateFactory::add_server_from_strings(
    std::shared_ptr<performance_test::Node> n,
    std::string srv_type,
    std::string service_name,
    rmw_qos_profile_t custom_qos_profile)
{

    const std::map<std::string, std::function<void()>>  servers_factory{
        {"10b",    [&] { n->add_server(performance_test::Service<performance_test_msgs::srv::Stamped10b>(service_name), custom_qos_profile); } }
    };

    if (servers_factory.find(srv_type) == servers_factory.end()){
        assert(0 && "unknown srv type passed to factory method!" );
    }

    servers_factory.at(srv_type)();
}


void performance_test::TemplateFactory::add_periodic_client_from_strings(
    std::shared_ptr<performance_test::Node> n,
    std::string srv_type,
    std::string service_name,
    rmw_qos_profile_t custom_qos_profile,
    std::chrono::milliseconds period_ms)
{

    const std::map<std::string, std::function<void()>>  clients_factory{
        {"10b",    [&] { n->add_periodic_client(performance_test::Service<performance_test_msgs::srv::Stamped10b>(service_name), period_ms, custom_qos_profile); } }
    };

    if (clients_factory.find(srv_type) == clients_factory.end()){
        assert(0 && "unknown srv type passed to factory method!" );
    }

    clients_factory.at(srv_type)();
}


size_t performance_test::TemplateFactory::get_msg_size(std::string msg_type, size_t msg_size)
{
    const std::map<std::string, size_t>  msg_factory{
        {"10b",         sizeof(performance_test_msgs::msg::Stamped10b().data) },
        {"100b",        sizeof(performance_test_msgs::msg::Stamped100b().data) },
        {"250b",        sizeof(performance_test_msgs::msg::Stamped250b().data) },
        {"1kb",         sizeof(performance_test_msgs::msg::Stamped1kb().data) },
        {"10kb",        sizeof(performance_test_msgs::msg::Stamped10kb().data) },
        {"100kb",       sizeof(performance_test_msgs::msg::Stamped100kb().data) },
        {"250kb",       sizeof(performance_test_msgs::msg::Stamped250kb().data) },
        {"1mb",         sizeof(performance_test_msgs::msg::Stamped1mb().data) },
        {"4mb",         sizeof(performance_test_msgs::msg::Stamped4mb().data) },
        {"8mb",         sizeof(performance_test_msgs::msg::Stamped8mb().data) },
        {"3float32",    sizeof(benchmark_msgs::msg::Stamped3Float32().data) },
        {"4float32",    sizeof(benchmark_msgs::msg::Stamped4Float32().data) },
        {"4int32",      sizeof(benchmark_msgs::msg::Stamped4Int32().data) },
        {"9float32",    sizeof(benchmark_msgs::msg::Stamped9Float32().data) },
        {"12float32",   sizeof(benchmark_msgs::msg::Stamped12Float32().data) },
        {"int64",       sizeof(benchmark_msgs::msg::StampedInt64().data) },
        {"vector",      msg_size }
    };

    if (msg_factory.find(msg_type) == msg_factory.end()){
        return 0;
    }

    return msg_factory.at(msg_type);
}


std::vector<std::shared_ptr<performance_test::Node>> performance_test::TemplateFactory::parse_topology_from_json(
    std::string json_path)
{

    std::vector<std::shared_ptr<performance_test::Node>> nodes_vec;

    std::ifstream ifs(json_path);

    // Check if file exists
    if(!ifs.good()) {
        std::cout << "ERROR. Can't find file: " << json_path << std::endl;
        return nodes_vec;
    }

    json j = json::parse(ifs);

    if (j.find("nodes") == j.end()){
        std::cout<<"ERROR. The provided json does not contain a nodes field"<<std::endl;
        return nodes_vec;
    }

    auto nodes_json = j["nodes"];

    for (auto n_json : nodes_json)
    {

        if (n_json.find("number") != n_json.end())
        {
            int number_of_nodes = n_json["number"];

            for(int node_number = 1 ; node_number <= number_of_nodes ; ++node_number)
            {
                std::string node_name_suffix = '_' + std::to_string(node_number);
                auto node = create_node_from_json(n_json, node_name_suffix);
                create_node_entities_from_json(node, n_json);
                nodes_vec.push_back(node);
            }
        }
        else
        {
            auto node = create_node_from_json(n_json);
            create_node_entities_from_json(node, n_json);
            nodes_vec.push_back(node);
        }

    }

    return nodes_vec;
}


std::shared_ptr<performance_test::Node> performance_test::TemplateFactory::create_node_from_json(
    const json node_json, std::string suffix)
{

    auto node_name = std::string(node_json["node_name"]) + suffix;
    auto node = this->create_node(node_name, _use_ipc, _verbose_mode, _ros2_namespace);

    return node;
}

void performance_test::TemplateFactory::create_node_entities_from_json(
    std::shared_ptr<performance_test::Node> node, const json node_json)
{

    if (node_json.find("publishers") != node_json.end()) {
        // if there is at least 1 publisher, add each of them
        for(auto p_json : node_json["publishers"]){
            this->add_periodic_publisher_from_json(node, p_json);
        }
    }

    if (node_json.find("subscribers") != node_json.end()) {
        // if there is at least 1 subscriber, add each of them
        for(auto s_json : node_json["subscribers"]){
            this->add_subscriber_from_json(node, s_json);
        }
    }

    if (node_json.find("clients") != node_json.end()) {
        // if there is at least 1 client, add each of them
        for(auto c_json : node_json["clients"]){
            this->add_periodic_client_from_json(node, c_json);
        }
    }

    if (node_json.find("servers") != node_json.end()) {
        // if there is at least 1 server, add each of them
        for(auto s_json : node_json["servers"]){
            this->add_server_from_json(node, s_json);
        }
    }

}

void performance_test::TemplateFactory::add_periodic_publisher_from_json(
    std::shared_ptr<performance_test::Node> node, const json pub_json)
{

    std::string topic_name = pub_json["topic_name"];
    std::string msg_type = pub_json["msg_type"];
    msg_pass_by_t msg_pass_by = UNIQUE_PTR;

    if (pub_json.find("msg_pass_by") != pub_json.end())
    {
        msg_pass_by = map_msg_pass_by[pub_json["msg_pass_by"]];
    }

    auto period_ms = std::chrono::milliseconds(pub_json["period_ms"]);
    size_t msg_size = 0;
    if (msg_type == "vector"){
        if (pub_json.find("msg_size") == pub_json.end())
        {
            assert(0 && "size of vector message not specified");
        }
        msg_size = pub_json["msg_size"];
    }

    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(pub_json);

    this->add_periodic_publisher_from_strings(
        node,
        msg_type,
        topic_name,
        msg_pass_by,
        custom_qos_profile,
        period_ms,
        msg_size);

}

void performance_test::TemplateFactory::add_subscriber_from_json(
    std::shared_ptr<performance_test::Node> node, const json sub_json)
{

    std::string topic_name = sub_json["topic_name"];
    std::string msg_type = sub_json["msg_type"];
    msg_pass_by_t msg_pass_by = SHARED_PTR;

    if (sub_json.find("msg_pass_by") != sub_json.end())
    {
        msg_pass_by = map_msg_pass_by[sub_json["msg_pass_by"]];
    }

    Tracker::TrackingOptions t_options;
    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(sub_json);

    this->add_subscriber_from_strings(
        node,
        msg_type,
        topic_name,
        t_options,
        msg_pass_by,
        custom_qos_profile);

}


void performance_test::TemplateFactory::add_periodic_client_from_json(
    std::shared_ptr<performance_test::Node> node, const json client_json)
{

    std::string service_name = client_json["service_name"];
    std::string srv_type = client_json["srv_type"];
    auto period_ms = std::chrono::milliseconds(client_json["period_ms"]);
    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(client_json);

    this->add_periodic_client_from_strings(
        node,
        srv_type,
        service_name,
        custom_qos_profile,
        period_ms);

}


void performance_test::TemplateFactory::add_server_from_json(
    std::shared_ptr<performance_test::Node> node, const json server_json)
{
    std::string service_name = server_json["service_name"];
    std::string srv_type = server_json["srv_type"];
    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(server_json);

    this->add_server_from_strings(
        node,
        srv_type,
        service_name,
        custom_qos_profile);

}


rmw_qos_profile_t performance_test::TemplateFactory::get_qos_from_json(
    const json entity_json)
{
    // Create custom QoS profile with default values
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // Crete map for each QoS
    std::map<std::string, rmw_qos_history_policy_t> history_qos_map{
        {"system_default", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
        {"keep_last",      RMW_QOS_POLICY_HISTORY_KEEP_LAST},
        {"keep_all",       RMW_QOS_POLICY_HISTORY_KEEP_ALL},
        {"unknown",        RMW_QOS_POLICY_HISTORY_UNKNOWN}
    };

    std::map<std::string, rmw_qos_reliability_policy_t> reliability_qos_map{
        {"system_default", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
        {"reliable",       RMW_QOS_POLICY_RELIABILITY_RELIABLE},
        {"best_effort",    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
        {"unknown",        RMW_QOS_POLICY_RELIABILITY_UNKNOWN}
    };

    std::map<std::string, rmw_qos_durability_policy_t> durability_qos_map{
        {"system_default",  RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
        {"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
        {"volatile",        RMW_QOS_POLICY_DURABILITY_VOLATILE},
        {"unknown",         RMW_QOS_POLICY_DURABILITY_UNKNOWN},
    };

    std::map<std::string, rmw_qos_liveliness_policy_t> liveliness_qos_map{
        {"system_default",  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT},
        {"automatic",       RMW_QOS_POLICY_LIVELINESS_AUTOMATIC},
        {"manual_by_node",  RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE},
        {"manual_by_topic", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC},
        {"unknown",         RMW_QOS_POLICY_LIVELINESS_UNKNOWN}
    };

    std::map<std::string, bool> namespace_conventions_qos_map{
        {"false", false},
        {"true",  true}
    };

    std::map<std::string, struct rmw_time_t> deadline_qos_map{
        {"default", RMW_QOS_DEADLINE_DEFAULT}
    };

    std::map<std::string, struct rmw_time_t> lifespan_qos_map{
        {"default", RMW_QOS_LIFESPAN_DEFAULT}
    };

    std::map<std::string, struct rmw_time_t> liveliness_lease_duration_qos_map{
        {"default", RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT}
    };

    // Look in the entity json file for QoS settings
    if (entity_json.find("qos_history") != entity_json.end())
    {
        custom_qos_profile.history = history_qos_map[entity_json["qos_history"]];
    }

    if (entity_json.find("qos_depth") != entity_json.end())
    {
        custom_qos_profile.depth = (size_t) entity_json["qos_depth"];
    }

    if (entity_json.find("qos_reliability") != entity_json.end())
    {
        custom_qos_profile.reliability = reliability_qos_map[entity_json["qos_reliability"]];
    }

    if (entity_json.find("qos_durability") != entity_json.end())
    {
        custom_qos_profile.durability = durability_qos_map[entity_json["qos_durability"]];
    }

    if (entity_json.find("qos_liveliness") != entity_json.end())
    {
        custom_qos_profile.liveliness = liveliness_qos_map[entity_json["qos_liveliness"]];
    }

    if (entity_json.find("qos_avoid_ros_namespace_conventions") != entity_json.end())
    {
        custom_qos_profile.avoid_ros_namespace_conventions = \
            namespace_conventions_qos_map[entity_json["qos_avoid_ros_namespace_conventions"]];
    }

    if (entity_json.find("qos_deadline") != entity_json.end())
    {
        custom_qos_profile.deadline = deadline_qos_map[entity_json["qos_deadline"]];
    }

    if (entity_json.find("qos_lifespan") != entity_json.end())
    {
        custom_qos_profile.lifespan = lifespan_qos_map[entity_json["qos_lifespan"]];
    }

    if (entity_json.find("qos_liveliness_lease_duration") != entity_json.end())
    {
      custom_qos_profile.liveliness_lease_duration = \
          liveliness_lease_duration_qos_map[entity_json["qos_liveliness_lease_duration"]];
    }

    return custom_qos_profile;
}
