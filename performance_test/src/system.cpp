/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <pthread.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "performance_test/system.hpp"
#include "performance_test/utils/names_utilities.hpp"
#include "performance_test/utils/stat_logger.hpp"
#include "performance_test/performance_node_base.hpp"
#include "performance_test/executors.hpp"
#include "performance_test/events_logger.hpp"

namespace performance_test {

static unsigned long int parse_line(std::string& line)
{
  std::string split_left = line.substr(0, line.find_first_of(" "));
  std::string split_right = line.substr(line.find_first_of(" "), line.length());
  line = split_right.substr(split_right.find_first_not_of(" "), split_right.length());
  return strtoul(split_left.c_str(), NULL, 0);
}

System::System(ExecutorType executor)
{
  _system_executor = executor;
}

void System::add_node(std::shared_ptr<performance_test::PerformanceNodeBase> node)
{
  if (_events_logger != nullptr) {
    node->set_events_logger(_events_logger);
  }

  int executor_id = node->get_executor_id();
  auto it = _executors_map.find(executor_id);
  if (it != _executors_map.end()) {
    auto & ex = it->second;
    ex.executor->add_node(node->get_node_base());
    ex.name = ex.name + "_" + node->get_node_name();
  } else {
    auto ex = NamedExecutor();

    switch (_system_executor)
    {
      case SINGLE_THREADED_EXECUTOR:
        ex.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        break;
      case STATIC_SINGLE_THREADED_EXECUTOR:
      default:
        ex.executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
        break;
    }

    ex.executor->add_node(node->get_node_base());
    ex.name = node->get_node_name();

    _executors_map.insert(std::make_pair(executor_id, ex));
  }

  _nodes.push_back(node);
}

void System::spin(int duration_sec, bool wait_for_discovery, bool name_threads)
{
  _experiment_duration_sec = duration_sec;
  // Store the instant when the experiment started
  _start_time = std::chrono::high_resolution_clock::now();

  // Check if some nodes have been added to this System
  if(_nodes.empty()) {
    assert(0 && "Error. Calling performance_test::System::spin when no nodes have been added.");
  }

  if (_events_logger != nullptr) {
    _events_logger->set_start_time(_start_time);
  }

  if (wait_for_discovery) {
    // wait until PDP and EDP are finished before starting
    // log events when each is completed
    this->wait_discovery();
  }

  for (const auto & pair : _executors_map) {
    auto & name = pair.second.name;
    auto & executor = pair.second.executor;

    // Spin each executor in a separate thread
    std::thread thread([=]() {
      executor->spin();
    });
    if(name_threads) {
      pthread_setname_np(thread.native_handle(), name.c_str());
    }
    thread.detach();
  }

  // let the nodes spin for the specified amount of time
  std::this_thread::sleep_for(std::chrono::seconds(_experiment_duration_sec));

  // after the timer, stop all the spin functions
  for (const auto & pair : _executors_map) {
    auto & executor = pair.second.executor;
    executor->cancel();
  }
}

void System::enable_events_logger(const std::string & events_logger_path)
{
  _events_logger = std::make_shared<EventsLogger>(events_logger_path);
}

void System::save_latency_all_stats(const std::string & filename) const
{
  if (filename.empty()) {
    std::cout << "[SystemLatencyLogger]: Error. Provided an empty filename." << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  std::ofstream out_file;
  out_file.open(filename);

  if(!out_file.is_open()) {
    std::cout << "[SystemLatencyLogger]: Error. Could not open file " << filename << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  performance_test::log_latency_all_stats(out_file, _nodes);
}

void System::save_latency_total_stats(const std::string & filename) const
{
  if (filename.empty()) {
    std::cout << "[SystemLatencyLogger]: Error. Provided an empty filename." << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  std::ofstream out_file;
  out_file.open(filename);

  if(!out_file.is_open()) {
    std::cout << "[SystemLatencyLogger]: Error. Could not open file " << filename << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  performance_test::log_latency_total_stats(out_file, _nodes);
}

void System::print_latency_all_stats() const
{
  performance_test::log_latency_all_stats(std::cout, _nodes);
}

void System::print_latency_total_stats() const
{
  performance_test::log_latency_total_stats(std::cout, _nodes);
}

void System::print_agregate_stats(const std::vector<std::string>& topology_json_list) const
{
  unsigned long int total_received = 0;
  unsigned long int total_lost = 0;
  unsigned long int total_late = 0;
  unsigned long int total_too_late = 0;
  unsigned long int total_latency = 0;

  for(const auto & json : topology_json_list) {
    std::string basename = json.substr(json.find_last_of("/") + 1, json.length());
    std::string filename = basename.substr(0, basename.length() - 5) + "_log/latency_total.txt";
    std::string line;
    std::ifstream log_file(filename);

    if (log_file.is_open()) {
      getline (log_file, line);
      // The second line contains the data to parse
      getline (log_file, line);

      total_received += parse_line(line);
      total_latency += parse_line(line);
      total_late += parse_line(line);
      parse_line(line);
      total_too_late += parse_line(line);
      parse_line(line);
      total_lost += parse_line(line);
      log_file.close();
    } else {
      std::cout << "[SystemLatencyLogger]: Error. Could not open file " << filename << std::endl;
    }
  }

  double average_latency = std::round(total_latency / topology_json_list.size());

  log_total_stats(total_received, total_lost, total_late, total_too_late,
    average_latency, std::cout);
}

void System::wait_discovery()
{
  // period at which PDP and EDP are checked
  std::chrono::milliseconds period = std::chrono::milliseconds(30);
  // maximum discovery time, after which the experiment is shut down
  std::chrono::milliseconds max_discovery_time = std::chrono::seconds(30);

  wait_pdp_discovery(period, max_discovery_time);

  wait_edp_discovery(period, max_discovery_time);
}

void System::wait_pdp_discovery(
  std::chrono::milliseconds period,
  std::chrono::milliseconds max_pdp_time)
{
  // period at which PDP is checked
  rclcpp::WallRate rate(period);

  auto pdp_start_time = std::chrono::high_resolution_clock::now();

  auto get_intersection_size = [=] (std::vector<std::string> A, std::vector<std::string> B) {
    // returns how many values are present in both A and B
    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());
    std::vector<std::string> v_intersection;
    std::set_intersection ( A.begin(), A.end(),
                            B.begin(), B.end(),
                            std::back_inserter(v_intersection));
    return v_intersection.size();
  };

  // create a vector with all the names of the nodes to be discovered
  std::vector<std::string> reference_names;
  for (const auto & n : _nodes) {
    std::string node_name = n->get_node_base()->get_fully_qualified_name();
    reference_names.push_back(node_name);
  }

  // count the total number of nodes
  size_t num_nodes = _nodes.size();

  bool pdp_ok = false;
  while (!pdp_ok) {
    for (const auto & n : _nodes) {
      // we use the intersection to avoid counting nodes discovered from other processes
      size_t discovered_participants = get_intersection_size(n->get_node_graph()->get_node_names(), reference_names);
      pdp_ok = (discovered_participants == num_nodes);
      if (!pdp_ok) { break; }
    }

    if (pdp_ok) { break; }

    // check if maximum discovery time exceeded
    auto t = std::chrono::high_resolution_clock::now();
    auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(t - pdp_start_time - max_pdp_time).count();
    if (duration > 0) {
      assert(0 && "[discovery] PDP took more than maximum discovery time");
    }

    rate.sleep();
  }

  if (_events_logger != nullptr) {
    // Create an event for PDP completed
    EventsLogger::Event pdp_ev;
    pdp_ev.caller_name = "SYSTEM";
    pdp_ev.code = EventsLogger::EventCode::discovery;
    pdp_ev.description = "[discovery] PDP completed";
    _events_logger->write_event(pdp_ev);
  }
}

void System::wait_edp_discovery(
  std::chrono::milliseconds period,
  std::chrono::milliseconds max_edp_time)
{
  // period at which EDP is checked
  rclcpp::WallRate rate(period);

  auto edp_start_time = std::chrono::high_resolution_clock::now();

  // count the number of subscribers for each topic
  std::map<std::string, int> subs_per_topic;
  for (const auto & n : _nodes) {
    auto trackers = n->sub_and_client_trackers();
    for (const auto & tracker : *trackers) {
      subs_per_topic[tracker.first] += 1;
    }
  }

  // TODO: the EDP should also take into account if subscriptions have been matched with publishers
  // This is needed in case of processes with only subscriptions
  bool edp_ok = false;
  while (!edp_ok) {
    for (const auto & n : _nodes) {
      auto published_topics = n->get_published_topics();
      // if the node has no publishers, it will be skipped.
      // however, the boolean flag has to be set to true.
      if (published_topics.empty()) {
        edp_ok = true;
        continue;
      }
      for (const auto & topic_name : published_topics) {
        int discovered_endpoints = n->get_node_graph()->count_subscribers(topic_name);
        // we check greater or equal to take into account for other processes
        edp_ok = (discovered_endpoints >= subs_per_topic[topic_name]);

        if (!edp_ok) {break;}
      }

      if (!edp_ok) {break;}
    }

    if (edp_ok) {break;}

    // check if maximum discovery time exceeded
    auto t = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(t - edp_start_time - max_edp_time).count();
    if (duration > 0) {
        assert(0 && "[discovery] EDP took more than maximum discovery time");
    }

    rate.sleep();
  }

  if (_events_logger != nullptr) {
    // Create an event for EDP completed
    EventsLogger::Event edp_ev;
    edp_ev.caller_name = "SYSTEM";
    edp_ev.code = EventsLogger::EventCode::discovery;
    edp_ev.description = "[discovery] EDP completed";
    _events_logger->write_event(edp_ev);
  }
}

}
