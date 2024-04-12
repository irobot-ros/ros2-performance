/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_HPP_
#define PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp/qos.hpp"

#include "performance_metrics/events_logger.hpp"
#include "performance_metrics/tracker.hpp"
#include "performance_test/communication.hpp"
#include "performance_test/utils/introspection.hpp"

namespace performance_test
{

struct NodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
};

class PerformanceNodeBase
{
public:
  using SharedPtr = std::shared_ptr<PerformanceNodeBase>;

  explicit PerformanceNodeBase(const NodeInterfaces & node_interfaces);

  virtual ~PerformanceNodeBase() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base();

  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph();

  rclcpp::Logger
  get_node_logger();

  const char *
  get_node_name();

  template<typename Msg>
  void add_subscriber(
    const std::string & topic_name,
    msg_pass_by_t msg_pass_by,
    performance_metrics::Tracker::Options tracking_options =
    performance_metrics::Tracker::Options(),
    const rclcpp::QoS & qos_profile = rclcpp::SensorDataQoS(),
    std::chrono::microseconds work_duration = std::chrono::microseconds::zero());

  template<typename Msg>
  void add_periodic_publisher(
    const std::string & topic_name,
    std::chrono::microseconds period,
    msg_pass_by_t msg_pass_by,
    const rclcpp::QoS & qos_profile = rclcpp::SensorDataQoS(),
    size_t size = 0);

  template<typename Msg>
  void add_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile = rclcpp::SensorDataQoS());

  template<typename Srv>
  void add_server(
    const std::string & service_name,
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  template<typename Srv>
  void add_periodic_client(
    const std::string & service_name,
    std::chrono::microseconds period,
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS(),
    size_t size = 0);

  template<typename Srv>
  void add_client(
    const std::string & service_name,
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  template<typename Action>
  void add_action_server(
    const std::string & action_name,
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  template<typename Action>
  void add_periodic_action_client(
    const std::string & action_name,
    std::chrono::microseconds period,
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  template<typename Action>
  void add_action_client(
    const std::string & action_name,
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  void add_timer(std::chrono::microseconds period, std::function<void()> callback);

  std::vector<performance_metrics::Tracker> sub_trackers();
  std::vector<performance_metrics::Tracker> service_trackers();
  std::vector<performance_metrics::Tracker> client_trackers();
  std::vector<performance_metrics::Tracker> action_client_trackers();
  std::vector<performance_metrics::Tracker> action_server_trackers();
  std::vector<performance_metrics::Tracker> pub_trackers();

  void set_events_logger(std::shared_ptr<performance_metrics::EventsLogger> ev);

  int get_executor_id();

  std::vector<std::string> get_published_topics();

protected:
  template<
    typename Msg,
    typename CallbackType = typename Msg::ConstSharedPtr>
  void add_subscriber_by_msg_variant(
    const std::string & topic_name,
    performance_metrics::Tracker::Options tracking_options =
    performance_metrics::Tracker::Options(),
    const rclcpp::QoS & qos_profile = rclcpp::SensorDataQoS(),
    std::chrono::microseconds work_duration = std::chrono::microseconds::zero());

  void store_subscription(
    rclcpp::SubscriptionBase::SharedPtr sub,
    const std::string & topic_name,
    const performance_metrics::Tracker::Options & tracking_options);

  void store_publisher(
    rclcpp::PublisherBase::SharedPtr pub,
    const std::string & topic_name,
    const performance_metrics::Tracker::Options & tracking_options);

  void store_client(
    rclcpp::ClientBase::SharedPtr client,
    const std::string & service_name,
    const performance_metrics::Tracker::Options & tracking_options);

  void store_server(
    rclcpp::ServiceBase::SharedPtr server,
    const std::string & service_name,
    const performance_metrics::Tracker::Options & tracking_options);

  void store_action_client(
    rclcpp_action::ClientBase::SharedPtr client,
    const std::string & action_name,
    const performance_metrics::Tracker::Options & tracking_options);

  void store_action_server(
    rclcpp_action::ServerBase::SharedPtr server,
    const std::string & action_name,
    const performance_metrics::Tracker::Options & tracking_options);

  performance_test_msgs::msg::PerformanceHeader create_msg_header(
    rclcpp::Time publish_time,
    float pub_frequency,
    uint32_t tracking_number,
    size_t msg_size);

  template<typename Msg>
  void publish_msg(
    const std::string & name,
    msg_pass_by_t msg_pass_by,
    size_t size,
    std::chrono::microseconds period);

  template<typename MsgT>
  typename std::enable_if<(msg_has_data_field<MsgT>::value), size_t>::type
  resize_msg(MsgT & msg, size_t size);

  template<typename MsgT>
  typename std::enable_if<(!msg_has_data_field<MsgT>::value), size_t>::type
  resize_msg(MsgT & msg, size_t size);

  template<typename DataT>
  typename std::enable_if<
    (!std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
  resize_data(DataT & data, size_t size);

  template<typename DataT>
  typename std::enable_if<
    (std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
  resize_data(DataT & data, size_t size);

  template<typename MsgType>
  void topic_callback(
    const std::string & topic_name,
    std::chrono::microseconds work_duration,
    MsgType msg);

  void handle_sub_received_msg(
    const std::string & topic_name,
    std::chrono::microseconds work_duration,
    const performance_test_msgs::msg::PerformanceHeader & msg_header);

  template<typename Srv>
  void send_request(const std::string & name, size_t size);

  template<typename Action>
  void send_action_goal_request(const std::string & name);

  template<typename Srv>
  void response_received_callback(
    const std::string & name,
    std::shared_ptr<typename Srv::Request> request,
    typename rclcpp::Client<Srv>::SharedFuture result_future);

  void handle_client_received_response(
    const std::string & service_name,
    const performance_test_msgs::msg::PerformanceHeader & request_header,
    const performance_test_msgs::msg::PerformanceHeader & response_header);

  template<typename Srv>
  void service_callback(
    const std::string & name,
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename Srv::Request> request,
    const std::shared_ptr<typename Srv::Response> response);

  performance_test_msgs::msg::PerformanceHeader
  handle_server_received_request(
    const std::string & service_name,
    const performance_test_msgs::msg::PerformanceHeader & request_header);

  // Client blocking call does not work with timers
  // Use a lock variable to avoid calling when you are already waiting
  std::atomic<bool> m_client_lock {false};
  std::atomic<bool> m_action_client_lock {false};

  NodeInterfaces m_node_interfaces;

  // A topic-name indexed map to store the publisher pointers with their
  // trackers.
  std::map<
    std::string,
    std::pair<rclcpp::PublisherBase::SharedPtr, performance_metrics::Tracker>> m_pubs;

  // A topic-name indexed map to store the subscriber pointers with their
  // trackers.
  std::map<
    std::string,
    std::pair<rclcpp::SubscriptionBase::SharedPtr, performance_metrics::Tracker>> m_subs;

  using ClientsTuple =
    std::tuple<rclcpp::ClientBase::SharedPtr, performance_metrics::Tracker, uint32_t>;

  using ActionClientsTuple =
    std::tuple<rclcpp_action::ClientBase::SharedPtr, performance_metrics::Tracker, uint32_t>;

  using ActionServersTuple =
    std::tuple<rclcpp_action::ServerBase::SharedPtr, performance_metrics::Tracker, uint32_t>;

  // A service-name indexed map to store the client pointers with their
  // trackers.
  std::map<std::string, ClientsTuple> m_clients;
  std::map<std::string, ActionClientsTuple> m_action_clients;
  std::map<std::string, ActionServersTuple> m_action_servers;

  // A service-name indexed map to store the server pointers with their
  // trackers.
  std::map<
    std::string,
    std::pair<rclcpp::ServiceBase::SharedPtr, performance_metrics::Tracker>> m_servers;

  std::vector<rclcpp::TimerBase::SharedPtr> m_timers;

  std::shared_ptr<performance_metrics::EventsLogger> m_events_logger;

  int m_executor_id = 0;
};

}  // namespace performance_test

#ifndef PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_IMPL_HPP_
// Template implementations
#include "performance_node_base_impl.hpp"
#endif

#endif  // PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_HPP_
