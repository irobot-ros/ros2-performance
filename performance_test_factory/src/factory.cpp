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
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nlohmann/json.hpp"
#include "performance_test/executors.hpp"
#include "performance_test/performance_node.hpp"
#include "performance_test_factory/factory.hpp"
#include "performance_test_factory/load_plugins.hpp"
#include "performance_test_factory/names_utilities.hpp"
#include "performance_test_factory/node_types.hpp"

using namespace std::chrono_literals;

namespace performance_test_factory
{

TemplateFactory::TemplateFactory(
  bool use_ipc,
  bool use_ros_params,
  bool verbose_mode,
  const std::string & ros2_namespace,
  NodeType node_type)
: m_use_ipc(use_ipc),
  m_use_ros_params(use_ros_params),
  m_verbose_mode(verbose_mode),
  m_ros2_namespace(ros2_namespace),
  m_node_type(node_type)
{}

performance_test::PerformanceNodeBase::SharedPtr
TemplateFactory::create_node(
  const std::string & name,
  bool use_ipc,
  bool use_ros_params,
  bool verbose,
  const std::string & ros2_namespace,
  int executor_id)
{
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(use_ipc);
  node_options.start_parameter_services(use_ros_params);
  node_options.start_parameter_event_publisher(use_ros_params);
  node_options.parameter_overrides({{"executor_id", executor_id}});

  performance_test::PerformanceNodeBase::SharedPtr node;

  switch (m_node_type) {
    case NodeType::RCLCPP_NODE:
      node = std::make_shared<performance_test::PerformanceNode<rclcpp::Node>>(
        name,
        ros2_namespace,
        node_options);
      break;
    case NodeType::RCLCPP_LIFECYCLE_NODE:
      node = std::make_shared<performance_test::PerformanceNode<rclcpp_lifecycle::LifecycleNode>>(
        name,
        ros2_namespace,
        node_options);
      break;
  }

  if (verbose) {
    auto ret = rcutils_logging_set_logger_level(
      node->get_node_logger().get_name(),
      RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      assert(0 && "Error setting logger verbosity");
    }
  }

  return node;
}

std::vector<performance_test::PerformanceNodeBase::SharedPtr>
TemplateFactory::create_subscriber_nodes(
  int start_id,
  int end_id,
  int n_publishers,
  const std::string & msg_type,
  performance_test::msg_pass_by_t msg_pass_by,
  const performance_metrics::Tracker::Options & tracking_options,
  const rclcpp::QoS & custom_qos_profile)
{
  std::vector<performance_test::PerformanceNodeBase::SharedPtr> nodes_vector;

  for (int node_id = start_id; node_id < end_id; node_id++) {
    std::string node_name = id_to_node_name(node_id);
    auto node = this->create_node(
      node_name,
      m_use_ipc,
      m_use_ros_params,
      m_verbose_mode,
      m_ros2_namespace);

    // TO DO: pass publisher list instead of n_publishers, to select the IDs (
    // default is a list from 0 to n_pubs or directly from n_subs to n_pubs)
    for (int k = 0; k < n_publishers; k++) {
      int topic_id = k + end_id;
      std::string topic_name = id_to_topic_name(topic_id);

      this->add_subscriber_from_strings(
        node,
        msg_type,
        topic_name,
        tracking_options,
        msg_pass_by,
        custom_qos_profile);
    }

    nodes_vector.push_back(node);
  }

  return nodes_vector;
}

std::vector<performance_test::PerformanceNodeBase::SharedPtr>
TemplateFactory::create_periodic_publisher_nodes(
  int start_id,
  int end_id,
  float frequency,
  const std::string & msg_type,
  performance_test::msg_pass_by_t msg_pass_by,
  size_t msg_size,
  const rclcpp::QoS & custom_qos_profile)
{
  std::vector<performance_test::PerformanceNodeBase::SharedPtr> nodes_vector;

  for (int node_id = start_id; node_id < end_id; node_id++) {
    std::string node_name = id_to_node_name(node_id);
    auto node = this->create_node(
      node_name,
      m_use_ipc,
      m_use_ros_params,
      m_verbose_mode,
      m_ros2_namespace);

    int topic_id = node_id;
    std::string topic_name = id_to_topic_name(topic_id);

    int period_us = (1000000 / frequency);
    std::chrono::microseconds period = std::chrono::microseconds(period_us);

    this->add_periodic_publisher_from_strings(
      node,
      msg_type,
      topic_name,
      msg_pass_by,
      custom_qos_profile,
      period,
      msg_size);

    nodes_vector.push_back(node);
  }

  return nodes_vector;
}

std::vector<performance_test::PerformanceNodeBase::SharedPtr>
TemplateFactory::create_periodic_client_nodes(
  int start_id,
  int end_id,
  int n_services,
  float frequency,
  const std::string & srv_type,
  const rclcpp::QoS & custom_qos_profile)
{
  std::vector<performance_test::PerformanceNodeBase::SharedPtr> nodes_vector;

  for (int node_id = start_id; node_id < end_id; node_id++) {
    std::string node_name = id_to_node_name(node_id);
    auto node = this->create_node(
      node_name,
      m_use_ipc,
      m_use_ros_params,
      m_verbose_mode,
      m_ros2_namespace);

    int period_us = (1000000 / frequency);
    std::chrono::microseconds period = std::chrono::microseconds(period_us);

    for (int k = 0; k < n_services; k++) {
      int service_id = k + end_id;
      std::string service_name = id_to_service_name(service_id);

      this->add_periodic_client_from_strings(
        node,
        srv_type,
        service_name,
        custom_qos_profile,
        period);
    }
    nodes_vector.push_back(node);
  }

  return nodes_vector;
}

std::vector<performance_test::PerformanceNodeBase::SharedPtr>
TemplateFactory::create_server_nodes(
  int start_id,
  int end_id,
  const std::string & srv_type,
  const rclcpp::QoS & custom_qos_profile)
{
  std::vector<performance_test::PerformanceNodeBase::SharedPtr> nodes_vector;

  for (int node_id = start_id; node_id < end_id; node_id++) {
    std::string node_name = id_to_node_name(node_id);
    auto node = this->create_node(
      node_name,
      m_use_ipc,
      m_use_ros_params,
      m_verbose_mode,
      m_ros2_namespace);

    int service_id = node_id;
    std::string service_name = id_to_service_name(service_id);

    this->add_server_from_strings(
      node,
      srv_type,
      service_name,
      custom_qos_profile);

    nodes_vector.push_back(node);
  }

  return nodes_vector;
}

void TemplateFactory::add_subscriber_from_strings(
  performance_test::PerformanceNodeBase::SharedPtr n,
  const std::string & msg_type,
  const std::string & topic_name,
  const performance_metrics::Tracker::Options & tracking_options,
  performance_test::msg_pass_by_t msg_pass_by,
  const rclcpp::QoS & custom_qos_profile)
{
  // Get library can modify the string, so we create a copy
  std::string msg_type__ = msg_type;
  auto library = performance_test_factory::get_library(msg_type__);

  typedef void (* function_impl_t)(
    performance_test::PerformanceNodeBase::SharedPtr,
    const std::string &,
    const std::string &,
    const performance_metrics::Tracker::Options &,
    performance_test::msg_pass_by_t,
    const rclcpp::QoS &);

  auto add_subscriber_impl =
    reinterpret_cast<function_impl_t>(library->get_symbol("add_subscriber_impl"));
  add_subscriber_impl(
    n,
    msg_type__,
    topic_name,
    tracking_options,
    msg_pass_by,
    custom_qos_profile);
}

void TemplateFactory::add_periodic_publisher_from_strings(
  performance_test::PerformanceNodeBase::SharedPtr n,
  const std::string & msg_type,
  const std::string & topic_name,
  performance_test::msg_pass_by_t msg_pass_by,
  const rclcpp::QoS & custom_qos_profile,
  std::chrono::microseconds period,
  size_t msg_size)
{
  // Get library can modify the string, so we create a copy
  std::string msg_type__ = msg_type;
  auto library = performance_test_factory::get_library(msg_type__);

  typedef void (* function_impl_t)(
    performance_test::PerformanceNodeBase::SharedPtr,
    const std::string &,
    const std::string &,
    performance_test::msg_pass_by_t,
    const rclcpp::QoS &,
    std::chrono::microseconds,
    size_t);

  auto add_publisher_impl =
    reinterpret_cast<function_impl_t>(library->get_symbol("add_publisher_impl"));
  add_publisher_impl(
    n,
    msg_type__,
    topic_name,
    msg_pass_by,
    custom_qos_profile,
    period,
    msg_size);
}

void TemplateFactory::add_periodic_client_from_strings(
  performance_test::PerformanceNodeBase::SharedPtr n,
  const std::string & srv_type,
  const std::string & service_name,
  const rclcpp::QoS & custom_qos_profile,
  std::chrono::microseconds period)
{
  // Get library can modify the string, so we create a copy
  std::string srv_type__ = srv_type;
  auto library = performance_test_factory::get_library(srv_type__);

  typedef void (* function_impl_t)(
    performance_test::PerformanceNodeBase::SharedPtr,
    const std::string &,
    const std::string &,
    const rclcpp::QoS &,
    std::chrono::microseconds);

  auto add_client_impl =
    reinterpret_cast<function_impl_t>(library->get_symbol("add_client_impl"));
  add_client_impl(
    n,
    srv_type__,
    service_name,
    custom_qos_profile,
    period);
}

void TemplateFactory::add_server_from_strings(
  performance_test::PerformanceNodeBase::SharedPtr n,
  const std::string & srv_type,
  const std::string & service_name,
  const rclcpp::QoS & custom_qos_profile)
{
  // Get library can modify the string, so we create a copy
  std::string srv_type__ = srv_type;
  auto library = performance_test_factory::get_library(srv_type__);

  typedef void (* function_impl_t)(
    performance_test::PerformanceNodeBase::SharedPtr,
    const std::string &,
    const std::string &,
    const rclcpp::QoS &);

  auto add_server_impl =
    reinterpret_cast<function_impl_t>(library->get_symbol("add_server_impl"));
  add_server_impl(
    n,
    srv_type__,
    service_name,
    custom_qos_profile);
}

void TemplateFactory::add_action_server_from_strings(
  performance_test::PerformanceNodeBase::SharedPtr n,
  const std::string & action_type,
  const std::string & action_name,
  const rclcpp::QoS & custom_qos_profile)
{
  // Get library can modify the string, so we create a copy
  std::string action_type__ = action_type;
  auto library = performance_test_factory::get_library(action_type__);

  typedef void (* function_impl_t)(
    performance_test::PerformanceNodeBase::SharedPtr,
    const std::string &,
    const std::string &,
    const rclcpp::QoS &);

  auto add_action_server_impl =
    reinterpret_cast<function_impl_t>(library->get_symbol("add_action_server_impl"));

  add_action_server_impl(
    n,
    action_type__,
    action_name,
    custom_qos_profile);
}

void TemplateFactory::add_periodic_action_client_from_strings(
  performance_test::PerformanceNodeBase::SharedPtr n,
  const std::string & action_type,
  const std::string & action_name,
  const rclcpp::QoS & custom_qos_profile,
  std::chrono::microseconds period)
{
  // Get library can modify the string, so we create a copy
  std::string action_type__ = action_type;
  auto library = performance_test_factory::get_library(action_type__);

  typedef void (* function_impl_t)(
    performance_test::PerformanceNodeBase::SharedPtr,
    const std::string &,
    const std::string &,
    const rclcpp::QoS &,
    std::chrono::microseconds);

  auto add_action_client_impl =
    reinterpret_cast<function_impl_t>(library->get_symbol("add_action_client_impl"));

  add_action_client_impl(
    n,
    action_type__,
    action_name,
    custom_qos_profile,
    period);
}

std::vector<performance_test::PerformanceNodeBase::SharedPtr>
TemplateFactory::parse_topology_from_json(
  const std::string & json_path,
  const performance_metrics::Tracker::Options & tracking_options)
{
  std::vector<performance_test::PerformanceNodeBase::SharedPtr> nodes_vec;

  std::ifstream ifs(json_path);
  // Check if file exists
  if (!ifs.good()) {
    std::cout << "ERROR. Can't find file: " << json_path << std::endl;
    return nodes_vec;
  }

  nlohmann::json j = nlohmann::json::parse(ifs);
  if (j.find("nodes") == j.end()) {
    std::cout << "ERROR. The provided json does not contain a nodes field" << std::endl;
    return nodes_vec;
  }

  auto nodes_json = j["nodes"];
  for (auto n_json : nodes_json) {
    if (n_json.find("number") != n_json.end()) {
      int number_of_nodes = n_json["number"];

      for (int node_number = 1; node_number <= number_of_nodes; node_number++) {
        std::string node_name_suffix = '_' + std::to_string(node_number);
        auto node = create_node_from_json(n_json, node_name_suffix);
        create_node_entities_from_json(node, n_json, tracking_options);
        nodes_vec.push_back(node);
      }
    } else {
      auto node = create_node_from_json(n_json);
      create_node_entities_from_json(node, n_json, tracking_options);
      nodes_vec.push_back(node);
    }
  }

  return nodes_vec;
}

performance_test::PerformanceNodeBase::SharedPtr
TemplateFactory::create_node_from_json(
  const nlohmann::json & node_json,
  const std::string & suffix)
{
  auto node_name = std::string(node_json["node_name"]) + suffix;
  auto node_namespace = m_ros2_namespace;
  if (node_json.find("node_namespace") != node_json.end()) {
    node_namespace += std::string(node_json["node_namespace"]) + suffix;
  }

  int executor_id = 0;
  if (node_json.find("executor_id") != node_json.end()) {
    executor_id = node_json["executor_id"];
  }

  auto node = this->create_node(
    node_name,
    m_use_ipc,
    m_use_ros_params,
    m_verbose_mode,
    node_namespace,
    executor_id);

  return node;
}

void TemplateFactory::create_node_entities_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & node_json,
  const performance_metrics::Tracker::Options & tracking_options)
{
  if (node_json.find("publishers") != node_json.end()) {
    // if there is at least 1 publisher, add each of them
    for (auto p_json : node_json["publishers"]) {
      this->add_periodic_publisher_from_json(node, p_json);
    }
  }

  if (node_json.find("subscribers") != node_json.end()) {
    // if there is at least 1 subscriber, add each of them
    for (auto s_json : node_json["subscribers"]) {
      this->add_subscriber_from_json(node, s_json, tracking_options);
    }
  }

  if (node_json.find("clients") != node_json.end()) {
    // if there is at least 1 client, add each of them
    for (auto c_json : node_json["clients"]) {
      this->add_periodic_client_from_json(node, c_json);
    }
  }

  if (node_json.find("servers") != node_json.end()) {
    // if there is at least 1 server, add each of them
    for (auto s_json : node_json["servers"]) {
      this->add_server_from_json(node, s_json);
    }
  }

  if (node_json.find("action_clients") != node_json.end()) {
    for (auto c_json : node_json["action_clients"]) {
      this->add_periodic_action_client_from_json(node, c_json);
    }
  }

  if (node_json.find("action_servers") != node_json.end()) {
    for (auto c_json : node_json["action_servers"]) {
      this->add_action_server_from_json(node, c_json);
    }
  }
}

void TemplateFactory::add_periodic_publisher_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & pub_json)
{
  std::string topic_name = pub_json["topic_name"];
  std::string msg_type = pub_json["msg_type"];

  float period_ms = 0;

  if (pub_json.find("freq_hz") != pub_json.end()) {
    float frequency = pub_json["freq_hz"];
    period_ms = 1000 / frequency;
  } else if (pub_json.find("period_ms") != pub_json.end()) {
    period_ms = pub_json["period_ms"];
  } else {
    std::cout << "Error! Publishers must set period_ms or freq_hz in json file" << std::endl;
  }

  auto period = std::chrono::microseconds(static_cast<int>(period_ms * 1000));

  size_t msg_size = 0;
  if (pub_json.find("msg_size") != pub_json.end()) {
    msg_size = pub_json["msg_size"];
  }

  rclcpp::QoS custom_qos_profile = get_qos_from_json(pub_json);

  auto msg_pass_by = get_msg_pass_by_from_json(
    pub_json,
    performance_test::msg_pass_by_t::PASS_BY_UNIQUE_PTR);

  this->add_periodic_publisher_from_strings(
    node,
    msg_type,
    topic_name,
    msg_pass_by,
    custom_qos_profile,
    period,
    msg_size);
}

void TemplateFactory::add_subscriber_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & sub_json,
  const performance_metrics::Tracker::Options & t_options)
{
  std::string topic_name = sub_json["topic_name"];
  std::string msg_type = sub_json["msg_type"];

  rclcpp::QoS custom_qos_profile = get_qos_from_json(sub_json);

  auto msg_pass_by = get_msg_pass_by_from_json(
    sub_json,
    performance_test::msg_pass_by_t::PASS_BY_SHARED_PTR);

  this->add_subscriber_from_strings(
    node,
    msg_type,
    topic_name,
    t_options,
    msg_pass_by,
    custom_qos_profile);
}

void TemplateFactory::add_periodic_client_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & client_json)
{
  std::string service_name = client_json["service_name"];
  std::string srv_type = client_json["srv_type"];

  float period_ms = 0;
  if (client_json.find("freq_hz") != client_json.end()) {
    float frequency = client_json["freq_hz"];
    period_ms = 1000 / frequency;
  } else if (client_json.find("period_ms") != client_json.end()) {
    period_ms = client_json["period_ms"];
  } else {
    std::cout << "Error! Clients must set period_ms or freq_hz in json file" << std::endl;
  }

  auto period = std::chrono::microseconds(static_cast<int>(period_ms * 1000));

  rclcpp::QoS custom_qos_profile = get_qos_from_json(client_json);

  this->add_periodic_client_from_strings(
    node,
    srv_type,
    service_name,
    custom_qos_profile,
    period);
}

void TemplateFactory::add_periodic_action_client_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & action_client_json)
{
  std::string action_name = action_client_json["action_name"];
  std::string action_type = action_client_json["action_type"];

  float period_ms = 0;
  if (action_client_json.find("freq_hz") != action_client_json.end()) {
    float frequency = action_client_json["freq_hz"];
    period_ms = 1000 / frequency;
  } else if (action_client_json.find("period_ms") != action_client_json.end()) {
    period_ms = action_client_json["period_ms"];
  } else {
    assert(0 && "Error! Action Clients must set period_ms or freq_hz in json file");
  }

  auto period = std::chrono::microseconds(static_cast<int>(period_ms * 1000));

  rclcpp::QoS custom_qos_profile = get_qos_from_json(action_client_json);

  this->add_periodic_action_client_from_strings(
    node,
    action_type,
    action_name,
    custom_qos_profile,
    period);
}

void TemplateFactory::add_action_server_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & action_server_json)
{
  std::string action_name = action_server_json["action_name"];
  std::string action_type = action_server_json["action_type"];
  rclcpp::QoS custom_qos_profile = get_qos_from_json(action_server_json);

  this->add_action_server_from_strings(
    node,
    action_type,
    action_name,
    custom_qos_profile);
}

void TemplateFactory::add_server_from_json(
  performance_test::PerformanceNodeBase::SharedPtr node,
  const nlohmann::json & server_json)
{
  std::string service_name = server_json["service_name"];
  std::string srv_type = server_json["srv_type"];
  rclcpp::QoS custom_qos_profile = get_qos_from_json(server_json);

  this->add_server_from_strings(
    node,
    srv_type,
    service_name,
    custom_qos_profile);
}

rclcpp::QoS TemplateFactory::get_qos_from_json(const nlohmann::json & entity_json)
{
  // Create custom QoS profile with default values
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  // Crete map for each QoS
  const std::map<std::string, rmw_qos_history_policy_t> history_qos_map{
    {"system_default", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
    {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
    {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL},
    {"unknown", RMW_QOS_POLICY_HISTORY_UNKNOWN}
  };

  const std::map<std::string, rmw_qos_reliability_policy_t> reliability_qos_map{
    {"system_default", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
    {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
    {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
    {"unknown", RMW_QOS_POLICY_RELIABILITY_UNKNOWN}
  };

  const std::map<std::string, rmw_qos_durability_policy_t> durability_qos_map{
    {"system_default", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
    {"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
    {"volatile", RMW_QOS_POLICY_DURABILITY_VOLATILE},
    {"unknown", RMW_QOS_POLICY_DURABILITY_UNKNOWN},
  };

  const std::map<std::string, rmw_qos_liveliness_policy_t> liveliness_qos_map{
    {"system_default", RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT},
    {"automatic", RMW_QOS_POLICY_LIVELINESS_AUTOMATIC},
    {"manual_by_topic", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC},
    {"unknown", RMW_QOS_POLICY_LIVELINESS_UNKNOWN}
  };

  const std::map<std::string, bool> namespace_conventions_qos_map{
    {"false", false},
    {"true", true}
  };

  const std::map<std::string, rmw_time_t> deadline_qos_map{
    {"default", RMW_QOS_DEADLINE_DEFAULT}
  };

  const std::map<std::string, rmw_time_t> lifespan_qos_map{
    {"default", RMW_QOS_LIFESPAN_DEFAULT}
  };

  const std::map<std::string, rmw_time_t> liveliness_lease_duration_qos_map{
    {"default", RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT}
  };

  // Look in the entity json file for QoS settings
  if (entity_json.find("qos_history") != entity_json.end()) {
    custom_qos_profile.history = history_qos_map.at(entity_json["qos_history"]);
  }

  if (entity_json.find("qos_depth") != entity_json.end()) {
    custom_qos_profile.depth = static_cast<size_t>(entity_json["qos_depth"]);
  }

  if (entity_json.find("qos_reliability") != entity_json.end()) {
    custom_qos_profile.reliability = reliability_qos_map.at(entity_json["qos_reliability"]);
  }

  if (entity_json.find("qos_durability") != entity_json.end()) {
    custom_qos_profile.durability = durability_qos_map.at(entity_json["qos_durability"]);
  }

  if (entity_json.find("qos_liveliness") != entity_json.end()) {
    custom_qos_profile.liveliness = liveliness_qos_map.at(entity_json["qos_liveliness"]);
  }

  if (entity_json.find("qos_avoid_ros_namespace_conventions") != entity_json.end()) {
    custom_qos_profile.avoid_ros_namespace_conventions =
      namespace_conventions_qos_map.at(entity_json["qos_avoid_ros_namespace_conventions"]);
  }

  if (entity_json.find("qos_deadline") != entity_json.end()) {
    custom_qos_profile.deadline = deadline_qos_map.at(entity_json["qos_deadline"]);
  }

  if (entity_json.find("qos_lifespan") != entity_json.end()) {
    custom_qos_profile.lifespan = lifespan_qos_map.at(entity_json["qos_lifespan"]);
  }

  if (entity_json.find("qos_liveliness_lease_duration") != entity_json.end()) {
    custom_qos_profile.liveliness_lease_duration =
      liveliness_lease_duration_qos_map.at(entity_json["qos_liveliness_lease_duration"]);
  }

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization::from_rmw(custom_qos_profile), custom_qos_profile);
  return qos;
}

performance_test::msg_pass_by_t TemplateFactory::get_msg_pass_by_from_json(
  const nlohmann::json & entity_json,
  performance_test::msg_pass_by_t default_value)
{
  auto msg_pass_by = default_value;
  if (entity_json.find("msg_pass_by") != entity_json.end()) {
    msg_pass_by = performance_test::string_to_msg_pass_by(entity_json["msg_pass_by"]);
  }

  return msg_pass_by;
}

}  // namespace performance_test_factory
