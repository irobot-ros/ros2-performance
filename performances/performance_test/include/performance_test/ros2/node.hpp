#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/ros2/communication.hpp"
#include "performance_test/ros2/tracker.hpp"
#include "performance_test/ros2/events_logger.hpp"

#include "performance_test_msgs/msg/stamped_vector.hpp"

using namespace std::chrono_literals;

namespace performance_test {

class Node : public rclcpp::Node
{

friend class System;

public:

  Node(const std::string& name, const std::string& ros2_namespace, bool ipc = true)
    : rclcpp::Node(name, ros2_namespace, ipc)
  {
    RCLCPP_INFO(this->get_logger(), "Node %s created", name.c_str());
  }


  template <typename Msg>
  void add_subscriber(const Topic<Msg>& topic, Tracker::TrackingOptions tracking_options = Tracker::TrackingOptions(),
                      rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {

    std::function<void(const typename Msg::SharedPtr msg)> callback_function = std::bind(
      &Node::_topic_callback<Msg>,
      this,
      topic.name,
      std::placeholders::_1
    );

    typename rclcpp::Subscription<Msg>::SharedPtr sub =
      this->create_subscription<Msg>(topic.name,
                                     callback_function,
                                     qos_profile);

    _subs.insert({ topic.name, { sub, Tracker(this->get_name(), topic.name, tracking_options) } });

    RCLCPP_INFO(this->get_logger(), "Subscriber to %s created", topic.name.c_str());
  }


  template <typename Msg>
  void add_periodic_publisher(const Topic<Msg>& topic,
                              std::chrono::milliseconds period,
                              rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
                              size_t size = 0)
  {

    this->add_publisher(topic, qos_profile);

    auto publisher_task = std::bind(
      &Node::_publish<Msg>,
      this,
      topic.name,
      size
    );

    // store the frequency of this publisher task
    _pubs.at(topic.name).second.set_frequency(1000 / period.count());

    this->add_timer(period, publisher_task);

  }


  template <typename Msg>
  void add_publisher(const Topic<Msg>& topic, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {

    typename rclcpp::Publisher<Msg>::SharedPtr pub = this->create_publisher<Msg>(topic.name, qos_profile);

    _pubs.insert({ topic.name, { pub, Tracker(this->get_name(), topic.name, Tracker::TrackingOptions()) } });

    RCLCPP_INFO(this->get_logger(),"Publisher to %s created", topic.name.c_str());
  }


  template <typename Srv>
  void add_server(const Service<Srv>& service, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {

    std::function<void(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename Srv::Request> request,
      const std::shared_ptr<typename Srv::Response> response)> callback_function = std::bind(
          &Node::_service_callback<Srv>,
          this,
          service.name,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3
    );

    typename rclcpp::Service<Srv>::SharedPtr server =
      this->create_service<Srv>(service.name,
                                callback_function,
                                qos_profile);

    _servers.insert({ service.name, { server, Tracker(this->get_name(), service.name, Tracker::TrackingOptions()) } });

    RCLCPP_INFO(this->get_logger(),"Server to %s created", service.name.c_str());
  }


  template <typename Srv>
  void add_periodic_client(const Service<Srv>& service,
                              std::chrono::milliseconds period,
                              rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
                              size_t size = 0)
  {

    this->add_client(service, qos_profile);

    std::function<void()> client_task = std::bind(
        &Node::_request<Srv>,
        this,
        service.name,
        size
      );

    // store the frequency of this client task
    _clients.at(service.name).second.set_frequency(1000 / period.count());

    this->add_timer(period, client_task);

  }

  template <typename Srv>
  void add_client(const Service<Srv>& service, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
  {

    typename rclcpp::Client<Srv>::SharedPtr client = this->create_client<Srv>(service.name, qos_profile);

    _clients.insert({ service.name, { client, Tracker(this->get_name(), service.name, Tracker::TrackingOptions()) } });

    RCLCPP_INFO(this->get_logger(),"Client to %s created", service.name.c_str());
  }


  void add_timer(std::chrono::milliseconds period, std::function< void() > callback)
  {

    rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(period, callback);

    _timers.push_back(timer);

  }

  // Return a vector with all the trackers
  typedef std::vector<std::pair<std::string, Tracker>> Trackers;
  std::shared_ptr<Trackers> all_trackers(bool all_interfaces = false)
  {
    auto trackers = std::make_shared<Trackers>();
    for(const auto& sub : _subs)
    {
      trackers->push_back({sub.first, sub.second.second});
    }

    for(const auto& client : _clients)
    {
      trackers->push_back({client.first, client.second.second});
    }

    // return also information from pubs and servers
    if (all_interfaces){
      for(const auto& pub : _pubs)
      {
        trackers->push_back({pub.first, pub.second.second});
      }

      for(const auto& server : _servers)
      {
        trackers->push_back({server.first, server.second.second});
      }
    }

    return trackers;
  }


  void set_events_logger(std::shared_ptr<EventsLogger> ev)
  {
    assert(ev != nullptr && "Called `Node::set_events_logger` passing a nullptr!");

    _events_logger = ev;
  }


private:

  template <typename Msg>
  void _publish(const std::string& name, size_t size)
  {
    // Get publisher and tracking count from map
    auto& pub_pair = _pubs.at(name);
    auto pub = std::static_pointer_cast<rclcpp::Publisher<Msg>>(pub_pair.first);
    auto& tracker = pub_pair.second;

    // Create message
    auto msg = std::make_shared<Msg>();
    // eventually resize the message (if it is dynamic) and measure its size
    _resize(msg, size);
    // get the frequency value that we stored when creating the publisher
    msg->header.frequency = tracker.frequency();
    // set the tracking count for this message
    msg->header.tracking_number = tracker.stat().n();
    //attach the timestamp as last operation before publishing
    msg->header.stamp = this->now();

    pub->publish(msg);
    // increase the tracker count (with 0 latency as this is the publisher)
    tracker.scan(msg->header, msg->header.stamp, _events_logger);
    RCLCPP_DEBUG(this->get_logger(), "Publishing to %s msg number %d", name.c_str(), msg->header.tracking_number);
  }

  // Only resize if it is a dynamic message
  template <typename Msg>
  typename std::enable_if<
    (!std::is_same<Msg, performance_test_msgs::msg::StampedVector>::value)>::type
  _resize(std::shared_ptr<Msg> msg, size_t size)
  {
    (void)size;
    msg->header.size = sizeof(msg->data);
    return;
  }

  template <typename Msg>
  typename std::enable_if<
    (std::is_same<Msg, performance_test_msgs::msg::StampedVector>::value)>::type
  _resize(std::shared_ptr<Msg> msg, size_t size)
  {
    msg->data.resize(size);
    msg->header.size = size;
    return;
  }


  template <typename Msg>
  void _topic_callback(const std::string& name, const typename Msg::SharedPtr msg)
  {
    // Scan new message's header
    auto& tracker = _subs.at(name).second;
    tracker.scan(msg->header, this->now(), _events_logger);

    RCLCPP_DEBUG(this->get_logger(), "Received on %s msg number %d after %f", name.c_str(), msg->header.tracking_number, tracker.stat().last());
  }


  template <typename Srv>
  void _request(const std::string& name, size_t size)
  {

    (void)size;

    if (_client_lock){
      return;
    }
    _client_lock = true;

    // Get client and tracking count from map
    auto& client_pair = _clients.at(name);
    auto client = std::static_pointer_cast<rclcpp::Client<Srv>>(client_pair.first);
    auto& tracker = client_pair.second;

    // Create request
    auto request = std::make_shared<typename Srv::Request>();
    // get the frequency value that we stored when creating the publisher
    request->header.frequency = tracker.frequency();
    request->header.tracking_number = tracker.stat().n();
    request->header.stamp = this->now();

    // Client non-blocking call + callback

    std::function<void(
      typename rclcpp::Client<Srv>::SharedFuture future)> callback_function = std::bind(
          &Node::_response_received_callback<Srv>,
          this,
          name,
          request,
          std::placeholders::_1
      );

    auto result_future = client->async_send_request(request, callback_function);

    // Client blocking call does not work with timers
    /*

    // send the request and wait for the response
    typename rclcpp::Client<Srv>::SharedFuture result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      // TODO: handle if request fails
      return;

    }
    tracker.scan(request->header, this->now(), _events_logger);
    */

    RCLCPP_DEBUG(this->get_logger(), "Requesting to %s request number %d", name.c_str(), request->header.tracking_number);
  }


  template <typename Srv>
  void _response_received_callback(const std::string& name, std::shared_ptr<typename Srv::Request> request, typename rclcpp::Client<Srv>::SharedFuture result_future)
  {
    _client_lock = false;

      // This is not used at the moment
      auto response = result_future.get();

      // Scan new message's header
      auto& tracker = _clients.at(name).second;
      tracker.scan(request->header, this->now(), _events_logger);

      RCLCPP_DEBUG(this->get_logger(), "Response on %s request number %d received with latency %f", name.c_str(), request->header.tracking_number, tracker.stat().last());
  }

  // Client blocking call does not work with timers
  // Use a lock variable to avoid calling when you are already waiting
  bool _client_lock = false;


  template <typename Srv>
  void _service_callback(
    const std::string& name,
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename Srv::Request> request,
    const std::shared_ptr<typename Srv::Response> response)
  {

    (void)request_header;

    // we use the tracker to store some information also on the server side
    auto& tracker = _servers.at(name).second;

    response->header.frequency = request->header.frequency;
    response->header.tracking_number = tracker.stat().n();
    response->header.stamp = this->now();

    tracker.scan(request->header, response->header.stamp, _events_logger);
    RCLCPP_DEBUG(this->get_logger(), "Request on %s request number %d received with latency %f", name.c_str(), request->header.tracking_number, tracker.stat().last());
  }


  // A topic-name indexed map to store the publisher pointers with their
  // trackers.
  std::map<std::string, std::pair<std::shared_ptr<void>, Tracker>> _pubs;

  // A topic-name indexed map to store the subscriber pointers with their
  // trackers.
  std::map<std::string, std::pair<std::shared_ptr<void>, Tracker>> _subs;

  // A service-name indexed map to store the client pointers with their
  // trackers.
  std::map<std::string, std::pair<std::shared_ptr<void>, Tracker>> _clients;

  // A service-name indexed map to store the server pointers with their
  // trackers.
  std::map<std::string, std::pair<std::shared_ptr<void>, Tracker>> _servers;


  std::vector<rclcpp::TimerBase::SharedPtr> _timers;

  std::shared_ptr<EventsLogger> _events_logger;

};
}