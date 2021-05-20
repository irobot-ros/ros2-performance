/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <iostream>

#include "rclcpp/time.hpp"

#include "performance_test/ros2/stat.hpp"
#include "performance_test/ros2/events_logger.hpp"

#include "performance_test_msgs/msg/performance_header.hpp"


namespace performance_test {

class Tracker
{
public:
  struct TrackingOptions {

      TrackingOptions(bool enable = true) : is_enabled(enable){};

      bool is_enabled = true;
      int late_percentage = 20;
      int late_absolute_us = 5000;
      int too_late_percentage = 100;
      int too_late_absolute_us = 50000;
  };

  typedef uint32_t TrackingNumber;

  Tracker() = delete;

  Tracker(std::string node_name, std::string topic_srv_name, TrackingOptions tracking_options)
  : _node_name(node_name), _topic_srv_name(topic_srv_name), _tracking_options(tracking_options) {};

  void scan(
    const performance_test_msgs::msg::PerformanceHeader& header,
    const rclcpp::Time& now,
    std::shared_ptr<EventsLogger> elog = nullptr);

  void add_sample(unsigned long latency_sample);

  TrackingNumber get_and_update_tracking_number();

  unsigned long int lost() const { return _lost_messages; }

  unsigned long int late() const { return _late_messages; }

  unsigned long int too_late() const { return _too_late_messages; }

  unsigned long int received() const { return _received_messages; }

  size_t size() const { return _size; }

  float frequency() const { return _frequency; }

  Stat<unsigned long int> stat() const { return _stat; }

  void set_frequency(float f) { _frequency = f; }

  unsigned long int last() const { return _last_latency; }

private:

  std::string _node_name;
  std::string _topic_srv_name;

  unsigned long int _last_latency = 0;
  unsigned long int _lost_messages = 0;
  unsigned long int _received_messages = 0;
  unsigned long int _late_messages = 0;
  unsigned long int _too_late_messages = 0;
  size_t _size = 0;
  float _frequency = 0;
  Stat<unsigned long int> _stat;
  TrackingNumber _tracking_number_count = 0;
  TrackingOptions _tracking_options;
};
}
