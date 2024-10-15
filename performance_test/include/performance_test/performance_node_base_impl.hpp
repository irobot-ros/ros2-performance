/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_IMPL_HPP_
#define PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_IMPL_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "performance_metrics/events_logger.hpp"
#include "performance_metrics/tracker.hpp"
#include "performance_test/communication.hpp"

#ifndef PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_HPP_
#include "performance_node_base.hpp"
#endif

namespace performance_test
{

template<typename Msg>
void PerformanceNodeBase::add_subscriber(
  const std::string & topic_name,
  msg_pass_by_t msg_pass_by,
  performance_metrics::Tracker::Options tracking_options,
  const rclcpp::QoS & qos_profile,
  std::chrono::microseconds work_duration)
{
  switch (msg_pass_by) {
    case msg_pass_by_t::PASS_BY_LOANED_MSG:
      RCLCPP_WARN(
        this->get_node_logger(),
        "Can't create sub '%s' using PASS_BY_LOANED_MSG; fallback to PASS_BY_SHARED_PTR",
        topic_name.c_str());
      [[fallthrough]];
    case msg_pass_by_t::PASS_BY_SHARED_PTR:
      add_subscriber_by_msg_variant<Msg, typename Msg::ConstSharedPtr>(
        topic_name,
        tracking_options,
        qos_profile,
        work_duration);
      break;
    case msg_pass_by_t::PASS_BY_UNIQUE_PTR:
      add_subscriber_by_msg_variant<Msg, typename Msg::UniquePtr>(
        topic_name,
        tracking_options,
        qos_profile,
        work_duration);
      break;
  }
}

template<
  typename Msg,
  typename CallbackType>
void PerformanceNodeBase::add_subscriber_by_msg_variant(
  const std::string & topic_name,
  performance_metrics::Tracker::Options tracking_options,
  const rclcpp::QoS & qos_profile,
  std::chrono::microseconds work_duration)
{
  std::function<void(CallbackType msg)> callback_function = std::bind(
    &PerformanceNodeBase::topic_callback<CallbackType>,
    this,
    topic_name,
    work_duration,
    std::placeholders::_1);

  rclcpp::SubscriptionBase::SharedPtr sub = rclcpp::create_subscription<Msg>(
    m_node_interfaces.parameters,
    m_node_interfaces.topics,
    topic_name,
    qos_profile,
    callback_function);

  this->store_subscription(sub, topic_name, tracking_options);
}

template<typename Msg>
void PerformanceNodeBase::add_periodic_publisher(
  const std::string & topic_name,
  std::chrono::microseconds period,
  msg_pass_by_t msg_pass_by,
  const rclcpp::QoS & qos_profile,
  size_t size)
{
  this->add_publisher<Msg>(topic_name, qos_profile);

  auto publisher_task = std::bind(
    &PerformanceNodeBase::publish_msg<Msg>,
    this,
    topic_name,
    msg_pass_by,
    size,
    period);

  this->add_timer(period, publisher_task);
}

template<typename Msg>
void PerformanceNodeBase::add_publisher(
  const std::string & topic_name,
  const rclcpp::QoS & qos_profile)
{
  rclcpp::PublisherBase::SharedPtr pub = rclcpp::create_publisher<Msg>(
    m_node_interfaces.topics,
    topic_name,
    qos_profile);

  this->store_publisher(pub, topic_name, performance_metrics::Tracker::Options());
}

template<typename Srv>
void PerformanceNodeBase::add_server(
  const std::string & service_name,
  const rclcpp::QoS & qos_profile)
{
  std::function<void(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename Srv::Request> request,
      const std::shared_ptr<typename Srv::Response> response)> callback_function = std::bind(
    &PerformanceNodeBase::service_callback<Srv>,
    this,
    service_name,
    std::placeholders::_1,
    std::placeholders::_2,
    std::placeholders::_3);

  rclcpp::ServiceBase::SharedPtr server = rclcpp::create_service<Srv>(
    m_node_interfaces.base,
    m_node_interfaces.services,
    service_name,
    callback_function,
    qos_profile.get_rmw_qos_profile(),
    nullptr);

  this->store_server(server, service_name, performance_metrics::Tracker::Options());
}

template<typename Srv>
void PerformanceNodeBase::add_periodic_client(
  const std::string & service_name,
  std::chrono::microseconds period,
  const rclcpp::QoS & qos_profile,
  size_t size)
{
  this->add_client<Srv>(service_name, qos_profile);

  std::function<void()> client_task = std::bind(
    &PerformanceNodeBase::send_request<Srv>,
    this,
    service_name,
    size);

  // store the frequency of this client task
  std::get<1>(m_clients.at(service_name)).set_frequency(1000000 / period.count());

  this->add_timer(period, client_task);
}

template<typename Srv>
void PerformanceNodeBase::add_client(
  const std::string & service_name,
  const rclcpp::QoS & qos_profile)
{
  rclcpp::ClientBase::SharedPtr client = rclcpp::create_client<Srv>(
    m_node_interfaces.base,
    m_node_interfaces.graph,
    m_node_interfaces.services,
    service_name,
    qos_profile,
    nullptr);

  this->store_client(client, service_name, performance_metrics::Tracker::Options());
}

template<typename Msg>
void PerformanceNodeBase::publish_msg(
  const std::string & name,
  msg_pass_by_t msg_pass_by,
  size_t size,
  std::chrono::microseconds period)
{
  // Get publisher and tracking count from map
  auto & pub_pair = m_pubs.at(name);
  auto pub = std::static_pointer_cast<rclcpp::Publisher<Msg>>(pub_pair.first);
  auto & tracker = pub_pair.second;

  float pub_frequency = 1000000.0 / period.count();

  auto tracking_number = tracker.get_and_update_tracking_number();
  rclcpp::Time publish_time;
  uint64_t pub_duration_us = 0;
  size_t msg_size = 0;
  switch (msg_pass_by) {
    case msg_pass_by_t::PASS_BY_SHARED_PTR:
      {
        // create a message and eventually resize it
        auto msg = std::make_shared<Msg>(rosidl_runtime_cpp::MessageInitialization::SKIP);
        msg_size = resize_msg(*msg, size);
        publish_time = m_node_interfaces.clock->get_clock()->now();

        msg->header = create_msg_header(
          publish_time,
          pub_frequency,
          tracking_number,
          msg_size);

        pub->publish(*msg);

        auto end_time = m_node_interfaces.clock->get_clock()->now();
        pub_duration_us = (end_time - publish_time).nanoseconds() / 1000.0f;

        break;
      }
    case msg_pass_by_t::PASS_BY_UNIQUE_PTR:
      {
        // create a message and eventually resize it
        auto msg = std::make_unique<Msg>(rosidl_runtime_cpp::MessageInitialization::SKIP);
        msg_size = resize_msg(*msg, size);
        publish_time = m_node_interfaces.clock->get_clock()->now();

        msg->header = create_msg_header(
          publish_time,
          pub_frequency,
          tracking_number,
          msg_size);

        pub->publish(std::move(msg));

        auto end_time = m_node_interfaces.clock->get_clock()->now();
        pub_duration_us = (end_time - publish_time).nanoseconds() / 1000.0f;

        break;
      }
    case msg_pass_by_t::PASS_BY_LOANED_MSG:
      {
        // create a message and eventually resize it
        auto loaned_msg = pub->borrow_loaned_message();
        auto & msg_ref = loaned_msg.get();
        msg_size = resize_msg(msg_ref, size);

        publish_time = m_node_interfaces.clock->get_clock()->now();

        msg_ref.header = create_msg_header(
          publish_time,
          pub_frequency,
          tracking_number,
          msg_size);

        pub->publish(std::move(loaned_msg));

        auto end_time = m_node_interfaces.clock->get_clock()->now();
        pub_duration_us = (end_time - publish_time).nanoseconds() / 1000.0f;

        break;
      }
  }

  tracker.add_sample(publish_time, pub_duration_us, msg_size, pub_frequency);

  RCLCPP_DEBUG(
    this->get_node_logger(),
    "Publishing to %s msg number %d took %lu us", name.c_str(), tracking_number, pub_duration_us);
}

template<typename MsgT>
typename std::enable_if<(msg_has_data_field<MsgT>::value), size_t>::type
PerformanceNodeBase::resize_msg(MsgT & msg, size_t size)
{
  return resize_data(msg.data, size);
}

template<typename MsgT>
typename std::enable_if<(!msg_has_data_field<MsgT>::value), size_t>::type
PerformanceNodeBase::resize_msg(MsgT & msg, size_t size)
{
  (void)size;
  return sizeof(msg);
}

template<typename DataT>
typename std::enable_if<
  (!std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
PerformanceNodeBase::resize_data(DataT & data, size_t size)
{
  // The payload is not a vector: nothing to resize
  (void)size;
  return sizeof(data);
}

template<typename DataT>
typename std::enable_if<
  (std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
PerformanceNodeBase::resize_data(DataT & data, size_t size)
{
  data.resize(size);
  return size;
}

template<typename MsgType>
void PerformanceNodeBase::topic_callback(
  const std::string & topic_name,
  std::chrono::microseconds work_duration,
  MsgType msg)
{
  this->handle_sub_received_msg(topic_name, work_duration, msg->header);
}

template<typename Srv>
void PerformanceNodeBase::send_request(const std::string & name, size_t size)
{
  (void)size;

  if (m_client_lock) {
    return;
  }
  m_client_lock = true;

  // Get client and tracking count from map
  auto & client_tuple = m_clients.at(name);
  auto client = std::static_pointer_cast<rclcpp::Client<Srv>>(std::get<0>(client_tuple));
  auto & tracker = std::get<1>(client_tuple);
  auto & tracking_number = std::get<2>(client_tuple);

  // Wait for service to come online
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    if (m_events_logger != nullptr) {
      // Create a descrption for the event
      std::stringstream description;
      description << "[service] '" << name.c_str() << "' unavailable after 1s";

      performance_metrics::EventsLogger::Event ev;
      ev.caller_name = name + "->" + m_node_interfaces.base->get_name();
      ev.code = performance_metrics::EventsLogger::EventCode::service_unavailable;
      ev.description = description.str();

      m_events_logger->write_event(ev);
    }
    m_client_lock = false;
    return;
  }

  // Create request
  auto request = std::make_shared<typename Srv::Request>();

  request->header = create_msg_header(
    m_node_interfaces.clock->get_clock()->now(),
    tracker.frequency(),
    tracking_number,
    0);

  // Client non-blocking call + callback

  std::function<void(
      typename rclcpp::Client<Srv>::SharedFuture future)> callback_function = std::bind(
    &PerformanceNodeBase::response_received_callback<Srv>,
    this,
    name,
    request,
    std::placeholders::_1);

  auto result_future = client->async_send_request(request, callback_function);
  tracking_number++;
  m_client_lock = false;

  // Client blocking call does not work with timers
  /*

  // send the request and wait for the response
  typename rclcpp::Client<Srv>::SharedFuture result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(m_node_interfaces.base_interface, result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    // TODO: handle if request fails
    return;

  }
  tracker.scan(request->header, m_node_interfaces.clock->get_clock()->now(), m_events_logger);
  */

  RCLCPP_DEBUG(
    this->get_node_logger(),
    "Requesting to %s request number %d", name.c_str(), request->header.tracking_number);
}

template<typename Srv>
void PerformanceNodeBase::response_received_callback(
  const std::string & name,
  std::shared_ptr<typename Srv::Request> request,
  typename rclcpp::Client<Srv>::SharedFuture result_future)
{
  // This is not used at the moment
  auto response = result_future.get();

  this->handle_client_received_response(name, request->header, response->header);
}

template<typename Srv>
void PerformanceNodeBase::service_callback(
  const std::string & name,
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<typename Srv::Request> request,
  const std::shared_ptr<typename Srv::Response> response)
{
  (void)request_header;
  response->header = this->handle_server_received_request(name, request->header);
}

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PERFORMANCE_NODE_BASE_IMPL_HPP_
