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
  : m_node_name(node_name), m_topic_srv_name(topic_srv_name), m_tracking_options(tracking_options)
  {}

  void scan(
    const performance_test_msgs::msg::PerformanceHeader & header,
    const rclcpp::Time & now,
    std::shared_ptr<EventsLogger> elog = nullptr);

  void add_sample(
    const rclcpp::Time & now,
    uint64_t latency_sample,
    size_t size,
    float frequency);

  uint32_t get_and_update_tracking_number();

  uint64_t lost() const {return m_lost_messages;}

  uint64_t late() const {return m_late_messages;}

  uint64_t too_late() const {return m_too_late_messages;}

  uint64_t received() const {return m_received_messages;}

  size_t size() const {return m_data_size;}

  float frequency() const {return m_frequency;}

  Stat<uint64_t> stat() const {return m_stat;}

  uint64_t delta_received() const {return m_delta_received_messages;}

  void reset_delta_received() const {m_delta_received_messages = 0;}

  Stat<uint64_t> delta_stat() const {return m_delta_stat;}

  void reset_delta_stat() const {m_delta_stat = Stat<uint64_t>();}

  double throughput() const;

  void set_frequency(float f) {m_frequency = f;}

  void set_size(size_t s) {m_data_size = s;}

  uint64_t last() const {return m_last_latency;}

  std::string get_node_name() const
  {
    return m_node_name;
  }

  std::string get_entity_name() const
  {
    return m_topic_srv_name;
  }

private:
  std::string m_node_name;
  std::string m_topic_srv_name;
  Options m_tracking_options;

  mutable Stat<uint64_t> m_delta_stat;
  mutable uint64_t m_delta_received_messages = 0;
  uint64_t m_last_latency = 0;
  uint64_t m_lost_messages = 0;
  uint64_t m_received_messages = 0;
  uint64_t m_late_messages = 0;
  uint64_t m_too_late_messages = 0;
  size_t m_data_size = 0;
  float m_frequency = 0;
  Stat<uint64_t> m_stat;
  uint32_t m_tracking_number_count = 0;

  rclcpp::Time m_first_msg_time;
  rclcpp::Time m_last_msg_time;
};

std::ostream & operator<<(std::ostream & os, const Tracker::Options & opts);

}  // namespace performance_metrics

#endif  // PERFORMANCE_METRICS__TRACKER_HPP_
