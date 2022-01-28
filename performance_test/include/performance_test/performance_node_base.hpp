/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "performance_test/communication.hpp"
#include "performance_test/events_logger.hpp"
#include "performance_test/tracker.hpp"

namespace performance_test
{

class PerformanceNodeBase
{
public:
  explicit PerformanceNodeBase(int executor_id = 0);

  virtual ~PerformanceNodeBase() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base();

  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph();

  rclcpp::Logger
  get_node_logger();

  const char *
  get_node_name();

  template<typename NodeT>
  void set_ros_node(NodeT* node)
  {
    m_node_base = node->template get_node_base_interface();
    m_node_clock = node->template get_node_clock_interface();
    m_node_graph = node->template get_node_graph_interface();
    m_node_logging = node->template get_node_logging_interface();
    m_node_parameters = node->template get_node_parameters_interface();
    m_node_services = node->template get_node_services_interface();
    m_node_timers = node->template get_node_timers_interface();
    m_node_topics = node->template get_node_topics_interface();
  }

  template <typename Msg>
  void add_subscriber(
    const std::string & topic_name,
    msg_pass_by_t msg_pass_by,
    Tracker::TrackingOptions tracking_options = Tracker::TrackingOptions(),
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {
    rclcpp::SubscriptionBase::SharedPtr sub;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);

    switch (msg_pass_by)
    {
      case PASS_BY_SHARED_PTR:
      {
        std::function<void(typename Msg::ConstSharedPtr msg)> callback_function = std::bind(
          &PerformanceNodeBase::_topic_callback<typename Msg::ConstSharedPtr>,
          this,
          topic_name,
          std::placeholders::_1);

        sub = rclcpp::create_subscription<Msg>(
          m_node_parameters,
          m_node_topics,
          topic_name,
          qos,
          callback_function);

        break;
      }

      case PASS_BY_UNIQUE_PTR:
      {
        std::function<void(typename Msg::UniquePtr msg)> callback_function = std::bind(
          &PerformanceNodeBase::_topic_callback<typename Msg::UniquePtr>,
          this,
          topic_name,
          std::placeholders::_1
        );

        sub = rclcpp::create_subscription<Msg>(
          m_node_parameters,
          m_node_topics,
          topic_name,
          qos,
          callback_function);

        break;
      }
    }

    this->store_subscription(sub, topic_name, tracking_options);
  }

  template <typename Msg>
  void add_periodic_publisher(
    const std::string & topic_name,
    std::chrono::microseconds period,
    msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
    size_t size = 0)
  {
    this->add_publisher<Msg>(topic_name, qos_profile);

    auto publisher_task = std::bind(
      &PerformanceNodeBase::_publish<Msg>,
      this,
      topic_name,
      msg_pass_by,
      size,
      period);

    this->add_timer(period, publisher_task);
  }

  template <typename Msg>
  void add_publisher(
    const std::string & topic_name,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);

    rclcpp::PublisherBase::SharedPtr pub = rclcpp::create_publisher<Msg>(
      m_node_topics,
      topic_name,
      qos);

    this->store_publisher(pub, topic_name, Tracker::TrackingOptions());
  }

  template <typename Srv>
  void add_server(
    const std::string & service_name,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {
    std::function<void(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename Srv::Request> request,
      const std::shared_ptr<typename Srv::Response> response)> callback_function = std::bind(
        &PerformanceNodeBase::_service_callback<Srv>,
        this,
        service_name,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
    );

    rclcpp::ServiceBase::SharedPtr server = rclcpp::create_service<Srv>(
      m_node_base,
      m_node_services,
      service_name,
      callback_function,
      qos_profile,
      nullptr);

    this->store_server(server, service_name, Tracker::TrackingOptions());
  }

  template <typename Srv>
  void add_periodic_client(
    const std::string & service_name,
    std::chrono::microseconds period,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
    size_t size = 0)
  {
    this->add_client<Srv>(service_name, qos_profile);

    std::function<void()> client_task = std::bind(
        &PerformanceNodeBase::_request<Srv>,
        this,
        service_name,
        size
      );

    // store the frequency of this client task
    std::get<1>(_clients.at(service_name)).set_frequency(1000000 / period.count());

    this->add_timer(period, client_task);
  }

  template <typename Srv>
  void add_client(
    const std::string & service_name,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {
    rclcpp::ClientBase::SharedPtr client = rclcpp::create_client<Srv>(
      m_node_base,
      m_node_graph,
      m_node_services,
      service_name,
      qos_profile,
      nullptr);

    this->store_client(client, service_name, Tracker::TrackingOptions());
  }

  void add_timer(std::chrono::microseconds period, std::function<void()> callback);

  using Trackers = std::vector<std::pair<std::string, Tracker>>;

  std::shared_ptr<Trackers> sub_and_client_trackers();

  std::shared_ptr<Trackers> pub_trackers();

  void set_events_logger(std::shared_ptr<EventsLogger> ev);

  int get_executor_id();

  std::vector<std::string> get_published_topics();

private:
  void store_subscription(
    rclcpp::SubscriptionBase::SharedPtr sub,
    const std::string & topic_name,
    const Tracker::TrackingOptions& tracking_options);

  void store_publisher(
    rclcpp::PublisherBase::SharedPtr pub,
    const std::string & topic_name,
    const Tracker::TrackingOptions& tracking_options);

  void store_client(
    rclcpp::ClientBase::SharedPtr client,
    const std::string & service_name,
    const Tracker::TrackingOptions& tracking_options);

  void store_server(
    rclcpp::ServiceBase::SharedPtr server,
    const std::string & service_name,
    const Tracker::TrackingOptions& tracking_options);

  performance_test_msgs::msg::PerformanceHeader create_msg_header(
    rclcpp::Time publish_time,
    float pub_frequency,
    Tracker::TrackingNumber tracking_number,
    size_t msg_size);

  template <typename Msg>
  void _publish(
    const std::string & name,
    msg_pass_by_t msg_pass_by,
    size_t size,
    std::chrono::microseconds period)
  {
    // Get publisher and tracking count from map
    auto & pub_pair = _pubs.at(name);
    auto pub = std::static_pointer_cast<rclcpp::Publisher<Msg>>(pub_pair.first);
    auto & tracker = pub_pair.second;

    float pub_frequency = 1000000.0 / period.count();

    auto tracking_number = tracker.get_and_update_tracking_number();
    unsigned long pub_time_us = 0;
    size_t msg_size = 0;
    switch (msg_pass_by)
    {
      case PASS_BY_SHARED_PTR:
      {
        // create a message and eventually resize it
        auto msg = std::make_shared<Msg>();
        msg_size = resize_msg(msg->data, size);
        auto publish_time = m_node_clock->get_clock()->now();

        msg->header = create_msg_header(
          publish_time,
          pub_frequency,
          tracking_number,
          msg_size);

        pub->publish(*msg);

        auto end_time = m_node_clock->get_clock()->now();
        pub_time_us = (end_time - publish_time).nanoseconds() / 1000.0f;

        break;
      }
      case PASS_BY_UNIQUE_PTR:
      {
        // create a message and eventually resize it
        auto msg = std::make_unique<Msg>();
        msg_size = resize_msg(msg->data, size);
        auto publish_time = m_node_clock->get_clock()->now();

        msg->header = create_msg_header(
          publish_time,
          pub_frequency,
          tracking_number,
          msg_size);

        pub->publish(std::move(msg));

        auto end_time = m_node_clock->get_clock()->now();
        pub_time_us = (end_time - publish_time).nanoseconds() / 1000.0f;

        break;
      }
    }

    tracker.set_frequency(pub_frequency);
    tracker.set_size(msg_size);
    tracker.add_sample(pub_time_us);

    RCLCPP_DEBUG(m_node_logging->get_logger(), "Publishing to %s msg number %d took %lu us", name.c_str(), tracking_number, pub_time_us);
  }

  template <typename DataT>
  typename std::enable_if<
    (!std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
  resize_msg(DataT & data, size_t size)
  {
    // The payload is not a vector: nothing to resize
    (void)size;
    return sizeof(data);
  }

  template <typename DataT>
  typename std::enable_if<
    (std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
  resize_msg(DataT & data, size_t size)
  {
    data.resize(size);
    return size;
  }

  template <typename MsgType>
  void _topic_callback(const std::string & name, MsgType msg)
  {
    this->_handle_sub_received_msg(name, msg->header);
  }

  void _handle_sub_received_msg(
    const std::string & topic_name,
    const performance_test_msgs::msg::PerformanceHeader& msg_header);

  template <typename Srv>
  void _request(const std::string & name, size_t size)
  {
    (void)size;

    if (_client_lock) {
      return;
    }
    _client_lock = true;

    // Get client and tracking count from map
    auto & client_tuple = _clients.at(name);
    auto client = std::static_pointer_cast<rclcpp::Client<Srv>>(std::get<0>(client_tuple));
    auto & tracker = std::get<1>(client_tuple);
    auto & tracking_number = std::get<2>(client_tuple);

    // Wait for service to come online
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      if (_events_logger != nullptr) {
        // Create a descrption for the event
        std::stringstream description;
        description << "[service] '"<< name.c_str() << "' unavailable after 1s";

        EventsLogger::Event ev;
        ev.caller_name = name + "->" + m_node_base->get_name();
        ev.code = EventsLogger::EventCode::service_unavailable;
        ev.description = description.str();

        _events_logger->write_event(ev);
      }
      _client_lock = false;
      return;
    }

    // Create request
    auto request = std::make_shared<typename Srv::Request>();

    request->header = create_msg_header(
      m_node_clock->get_clock()->now(),
      tracker.frequency(),
      tracking_number,
      0);

    // Client non-blocking call + callback

    std::function<void(
      typename rclcpp::Client<Srv>::SharedFuture future)> callback_function = std::bind(
        &PerformanceNodeBase::_response_received_callback<Srv>,
        this,
        name,
        request,
        std::placeholders::_1
      );

    auto result_future = client->async_send_request(request, callback_function);
    tracking_number++;
    _client_lock = false;

    // Client blocking call does not work with timers
    /*

    // send the request and wait for the response
    typename rclcpp::Client<Srv>::SharedFuture result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(m_node_base_interface, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      // TODO: handle if request fails
      return;

    }
    tracker.scan(request->header, m_node_clock->get_clock()->now(), _events_logger);
    */

    RCLCPP_DEBUG(m_node_logging->get_logger(), "Requesting to %s request number %d", name.c_str(), request->header.tracking_number);
  }

  template <typename Srv>
  void _response_received_callback(const std::string & name, std::shared_ptr<typename Srv::Request> request, typename rclcpp::Client<Srv>::SharedFuture result_future)
  {
    // This is not used at the moment
    auto response = result_future.get();

    this->_handle_client_received_response(name, request->header, response->header);
  }

  void _handle_client_received_response(
    const std::string & service_name,
    const performance_test_msgs::msg::PerformanceHeader& request_header,
    const performance_test_msgs::msg::PerformanceHeader& response_header);

  // Client blocking call does not work with timers
  // Use a lock variable to avoid calling when you are already waiting
  bool _client_lock = false;

  template <typename Srv>
  void _service_callback(
    const std::string & name,
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename Srv::Request> request,
    const std::shared_ptr<typename Srv::Response> response)
  {
    (void)request_header;
    response->header = this->_handle_server_received_request(name, request->header);
  }

  performance_test_msgs::msg::PerformanceHeader
  _handle_server_received_request(
    const std::string & service_name,
    const performance_test_msgs::msg::PerformanceHeader& request_header);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr m_node_base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr m_node_graph;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr m_node_logging;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr m_node_timers;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr m_node_topics;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr m_node_services;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr m_node_clock;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr m_node_parameters;

  // A topic-name indexed map to store the publisher pointers with their
  // trackers.
  std::map<std::string, std::pair<rclcpp::PublisherBase::SharedPtr, Tracker>> _pubs;

  // A topic-name indexed map to store the subscriber pointers with their
  // trackers.
  std::map<std::string, std::pair<rclcpp::SubscriptionBase::SharedPtr, Tracker>> _subs;

  // A service-name indexed map to store the client pointers with their
  // trackers.
  std::map<std::string, std::tuple<rclcpp::ClientBase::SharedPtr, Tracker, Tracker::TrackingNumber>> _clients;

  // A service-name indexed map to store the server pointers with their
  // trackers.
  std::map<std::string, std::pair<rclcpp::ServiceBase::SharedPtr, Tracker>> _servers;

  std::vector<rclcpp::TimerBase::SharedPtr> _timers;

  std::shared_ptr<EventsLogger> _events_logger;

  int m_executor_id = 0;
};

}
