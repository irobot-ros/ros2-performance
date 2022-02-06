/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_METRICS__TRACKER_HPP_
#define PERFORMANCE_METRICS__TRACKER_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/time.hpp"

#include "performance_metrics/stat.hpp"
#include "performance_metrics/events_logger.hpp"

#include "performance_test_msgs/msg/performance_header.hpp"

namespace performance_metrics
{

class Tracker
{
public:
  struct Options
  {
    explicit Options(bool enable = true)
    : is_enabled(enable)
    {}

    bool is_enabled = true;
    int late_percentage = 20;
    int late_absolute_us = 5000;
    int too_late_percentage = 100;
    int too_late_absolute_us = 50000;
  };

  Tracker() = delete;

  Tracker(
    const std::string & node_name,
    const std::string & topic_srv_name,
    const Options & tracking_options)
  : _node_name(node_name), _topic_srv_name(topic_srv_name), _tracking_options(tracking_options)
  {}

  void scan(
    const performance_test_msgs::msg::PerformanceHeader & header,
    const rclcpp::Time & now,
    std::shared_ptr<EventsLogger> elog = nullptr);

  void add_sample(uint64_t latency_sample);

  uint32_t get_and_update_tracking_number();

  uint64_t lost() const {return _lost_messages;}

  uint64_t late() const {return _late_messages;}

  uint64_t too_late() const {return _too_late_messages;}

  uint64_t received() const {return _received_messages;}

  size_t size() const {return _size;}

  float frequency() const {return m_frequency;}

  Stat<uint64_t> stat() const {return _stat;}

  void set_frequency(float f) {m_frequency = f;}

  void set_size(size_t s) {_size = s;}

  uint64_t last() const {return _last_latency;}

  std::string get_node_name() const
  {
    return _node_name;
  }

  std::string get_entity_name() const
  {
    return _topic_srv_name;
  }

private:
  std::string _node_name;
  std::string _topic_srv_name;

  uint64_t _last_latency = 0;
  uint64_t _lost_messages = 0;
  uint64_t _received_messages = 0;
  uint64_t _late_messages = 0;
  uint64_t _too_late_messages = 0;
  size_t _size = 0;
  float m_frequency = 0;
  Stat<uint64_t> _stat;
  uint32_t _tracking_number_count = 0;
  Options _tracking_options;
};

}  // namespace performance_metrics

#endif  // PERFORMANCE_METRICS__TRACKER_HPP_
