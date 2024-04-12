/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "performance_metrics/dummy_work.hpp"
#include "performance_metrics/events_logger.hpp"
#include "performance_metrics/tracker.hpp"
#include "performance_test/communication.hpp"
#include "performance_test/performance_node_base.hpp"

using namespace std::chrono_literals;

namespace performance_test
{

PerformanceNodeBase::PerformanceNodeBase(const NodeInterfaces & node_interfaces)
: m_node_interfaces(node_interfaces)
{
  m_executor_id =
    m_node_interfaces.parameters->declare_parameter(
    "executor_id", rclcpp::ParameterValue(0)
    ).get<int>();

  RCLCPP_INFO(
    this->get_node_logger(),
    "PerformanceNode %s created with executor id %d", this->get_node_name(), m_executor_id);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
PerformanceNodeBase::get_node_base()
{
  return m_node_interfaces.base;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
PerformanceNodeBase::get_node_graph()
{
  return m_node_interfaces.graph;
}

rclcpp::Logger
PerformanceNodeBase::get_node_logger()
{
  return m_node_interfaces.logging->get_logger();
}

const char *
PerformanceNodeBase::get_node_name()
{
  return m_node_interfaces.base->get_name();
}

void PerformanceNodeBase::add_timer(
  std::chrono::microseconds period,
  std::function<void()> callback)
{
  rclcpp::TimerBase::SharedPtr timer = rclcpp::create_wall_timer(
    period,
    callback,
    nullptr,
    m_node_interfaces.base.get(),
    m_node_interfaces.timers.get());

  m_timers.push_back(timer);
}

std::vector<performance_metrics::Tracker>
PerformanceNodeBase::sub_trackers()
{
  std::vector<performance_metrics::Tracker> trackers;
  for (const auto & sub : m_subs) {
    trackers.push_back(sub.second.second);
  }

  return trackers;
}

std::vector<performance_metrics::Tracker> PerformanceNodeBase::client_trackers()
{
  std::vector<performance_metrics::Tracker> trackers;
  for (const auto & client : m_clients) {
    trackers.push_back(std::get<1>(client.second));
  }

  return trackers;
}

std::vector<performance_metrics::Tracker> PerformanceNodeBase::service_trackers()
{
  std::vector<performance_metrics::Tracker> trackers;
  for (const auto & service : m_servers) {
    trackers.push_back(std::get<1>(service.second));
  }

  return trackers;
}

std::vector<performance_metrics::Tracker> PerformanceNodeBase::action_client_trackers()
{
  std::vector<performance_metrics::Tracker> trackers;
  for (const auto & client : m_action_clients) {
    trackers.push_back(std::get<1>(client.second));
  }

  return trackers;
}

std::vector<performance_metrics::Tracker> PerformanceNodeBase::action_server_trackers()
{
  std::vector<performance_metrics::Tracker> trackers;
  for (const auto & client : m_action_servers) {
    trackers.push_back(std::get<1>(client.second));
  }

  return trackers;
}

std::vector<performance_metrics::Tracker> PerformanceNodeBase::pub_trackers()
{
  std::vector<performance_metrics::Tracker> trackers;
  for (const auto & pub : m_pubs) {
    trackers.push_back(pub.second.second);
  }

  return trackers;
}

void PerformanceNodeBase::set_events_logger(
  std::shared_ptr<performance_metrics::EventsLogger> ev)
{
  assert(ev != nullptr && "Called `PerformanceNode::set_events_logger` passing a nullptr!");

  m_events_logger = ev;
}

int PerformanceNodeBase::get_executor_id()
{
  return m_executor_id;
}

std::vector<std::string> PerformanceNodeBase::get_published_topics()
{
  std::vector<std::string> topics;

  for (const auto & pub_tracker : m_pubs) {
    std::string topic_name = pub_tracker.first;
    topics.push_back(topic_name);
  }

  return topics;
}

void PerformanceNodeBase::store_subscription(
  rclcpp::SubscriptionBase::SharedPtr sub,
  const std::string & topic_name,
  const performance_metrics::Tracker::Options & tracking_options)
{
  auto tracker = performance_metrics::Tracker(
    m_node_interfaces.base->get_name(),
    topic_name,
    tracking_options);

  m_subs.insert({topic_name, {sub, tracker}});
  RCLCPP_INFO(this->get_node_logger(), "Subscription to %s created", topic_name.c_str());
}

void PerformanceNodeBase::store_publisher(
  rclcpp::PublisherBase::SharedPtr pub,
  const std::string & topic_name,
  const performance_metrics::Tracker::Options & tracking_options)
{
  auto tracker = performance_metrics::Tracker(
    m_node_interfaces.base->get_name(),
    topic_name,
    tracking_options);

  m_pubs.insert({topic_name, {pub, tracker}});
  RCLCPP_INFO(this->get_node_logger(), "Publisher to %s created", topic_name.c_str());
}

void PerformanceNodeBase::store_client(
  rclcpp::ClientBase::SharedPtr client,
  const std::string & service_name,
  const performance_metrics::Tracker::Options & tracking_options)
{
  auto tracker = performance_metrics::Tracker(
    m_node_interfaces.base->get_name(),
    service_name,
    tracking_options);

  m_clients.insert(
    {
      service_name,
      std::tuple<std::shared_ptr<rclcpp::ClientBase>, performance_metrics::Tracker, uint32_t>{
        client,
        tracker,
        0
      }
    });

  RCLCPP_INFO(this->get_node_logger(), "Client to %s created", service_name.c_str());
}

void PerformanceNodeBase::store_action_client(
  rclcpp_action::ClientBase::SharedPtr client,
  const std::string & action_name,
  const performance_metrics::Tracker::Options & tracking_options)
{
  auto tracker = performance_metrics::Tracker(
    m_node_interfaces.base->get_name(),
    action_name,
    tracking_options);

  m_action_clients.insert(
    {
      action_name,
      std::make_tuple(client, tracker, 0)
    });

  RCLCPP_INFO(this->get_node_logger(), "Action Client to %s created", action_name.c_str());
}

void PerformanceNodeBase::store_action_server(
  rclcpp_action::ServerBase::SharedPtr server,
  const std::string & action_name,
  const performance_metrics::Tracker::Options & tracking_options)
{
  auto tracker = performance_metrics::Tracker(
    m_node_interfaces.base->get_name(),
    action_name,
    tracking_options);

  m_action_servers.insert(
  {
    action_name,
    std::make_tuple(server, tracker, 0)
  });

  RCLCPP_INFO(this->get_node_logger(), "Action Server to %s created", action_name.c_str());
}

void PerformanceNodeBase::store_server(
  rclcpp::ServiceBase::SharedPtr server,
  const std::string & service_name,
  const performance_metrics::Tracker::Options & tracking_options)
{
  auto tracker = performance_metrics::Tracker(
    m_node_interfaces.base->get_name(),
    service_name,
    tracking_options);

  m_servers.insert({service_name, {server, tracker}});
  RCLCPP_INFO(this->get_node_logger(), "Server to %s created", service_name.c_str());
}

performance_test_msgs::msg::PerformanceHeader
PerformanceNodeBase::create_msg_header(
  rclcpp::Time publish_time,
  float pub_frequency,
  uint32_t tracking_number,
  size_t msg_size)
{
  performance_test_msgs::msg::PerformanceHeader header;

  header.size = msg_size;
  // get the frequency value that we stored when creating the publisher
  header.frequency = pub_frequency;
  // set the tracking count for this message
  header.tracking_number = tracking_number;
  // attach the timestamp as last operation before publishing
  header.stamp = publish_time;

  return header;
}

void PerformanceNodeBase::handle_sub_received_msg(
  const std::string & topic_name,
  std::chrono::microseconds work_duration,
  const performance_test_msgs::msg::PerformanceHeader & msg_header)
{
  // Scan new message's header
  auto & tracker = m_subs.at(topic_name).second;
  tracker.scan(msg_header, m_node_interfaces.clock->get_clock()->now(), m_events_logger);

  RCLCPP_DEBUG(
    this->get_node_logger(),
    "Received on %s msg number %d after %lu us",
    topic_name.c_str(),
    msg_header.tracking_number,
    tracker.last());

  // Perform some dummy work to simulate data processing
  performance_metrics::dummy_work(work_duration);
}

void PerformanceNodeBase::handle_client_received_response(
  const std::string & service_name,
  const performance_test_msgs::msg::PerformanceHeader & request_header,
  const performance_test_msgs::msg::PerformanceHeader & response_header)
{
  (void)response_header;

  auto & tracker = std::get<1>(m_clients.at(service_name));
  tracker.scan(request_header, m_node_interfaces.clock->get_clock()->now(), m_events_logger);

  RCLCPP_DEBUG(
    this->get_node_logger(),
    "Response on %s request number %d received after %lu us",
    service_name.c_str(),
    request_header.tracking_number,
    tracker.last());
}

performance_test_msgs::msg::PerformanceHeader
PerformanceNodeBase::handle_server_received_request(
  const std::string & service_name,
  const performance_test_msgs::msg::PerformanceHeader & request_header)
{
  // we use the tracker to store some information also on the server side
  auto & tracker = m_servers.at(service_name).second;

  auto response_header = this->create_msg_header(
    m_node_interfaces.clock->get_clock()->now(),
    request_header.frequency,
    tracker.stat().n(),
    0);

  tracker.scan(request_header, response_header.stamp, m_events_logger);
  RCLCPP_DEBUG(
    this->get_node_logger(),
    "Request on %s request number %d received %lu us",
    service_name.c_str(),
    request_header.tracking_number,
    tracker.last());

  return response_header;
}

}  // namespace performance_test
