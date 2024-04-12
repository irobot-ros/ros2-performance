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

#include "performance_metrics/events_logger.hpp"
#include "performance_metrics/stat_logger.hpp"
#include "performance_test/system.hpp"
#include "performance_test/performance_node_base.hpp"
#include "performance_test/executors.hpp"

namespace performance_test
{

static uint64_t parse_line(std::string & line, const bool csv_out)
{
  std::string sep = (csv_out) ? "," : " ";
  size_t sep_pos = line.find_first_of(sep);

  if (sep_pos == std::string::npos) {
    std::cerr << "Separator not found in line: " << line << std::endl;
    return 0;
  }

  std::string split_left = line.substr(0, sep_pos);
  std::string split_right = line.substr(sep_pos, line.length());

  size_t non_sep_pos = split_right.find_first_not_of(sep);
  if (non_sep_pos == std::string::npos) {
    std::cerr << "Non-separator character not found after separator in line: " << line << std::endl;
    return 0; // or handle the error appropriately
  }

  line = split_right.substr(non_sep_pos, split_right.length());

  return strtoul(split_left.c_str(), NULL, 0);
}

System::System(
  ExecutorType executor_type,
  SpinType spin_type,
  const std::optional<std::string> & events_logger_path,
  const bool csv_out)
{
  m_executor_type = executor_type;
  m_spin_type = spin_type;
  m_csv_out = csv_out;
  if (events_logger_path) {
    m_events_logger =
      std::make_shared<performance_metrics::EventsLogger>(*events_logger_path, m_csv_out);
  }
}

System::~System()
{
  for (auto & pair : m_executors_map) {
    auto & executor = pair.second.executor;
    executor->cancel();
  }

  for (auto & thread : m_threads) {
    thread->join();
  }
}

void System::add_node(std::shared_ptr<performance_test::PerformanceNodeBase> node)
{
  if (m_events_logger != nullptr) {
    node->set_events_logger(m_events_logger);
  }

  int executor_id = node->get_executor_id();
  auto it = m_executors_map.find(executor_id);
  if (it != m_executors_map.end()) {
    // An executor with this ID has already been created, so add node
    auto & ex = it->second;
    ex.executor->add_node(node->get_node_base());
    ex.name = ex.name + "_" + node->get_node_name();
  } else {
    // Create a new executor with this ID
    auto ex = NamedExecutor();
    ex.executor = performance_test::make_executor(m_executor_type);
    ex.executor->add_node(node->get_node_base());
    ex.name = node->get_node_name();

    m_executors_map.insert(std::make_pair(executor_id, ex));
  }

  m_nodes.push_back(node);
}

void System::spin(std::chrono::seconds duration, bool wait_for_discovery)
{
  m_experiment_duration = duration;
  // Store the instant when the experiment started
  m_start_time = std::chrono::high_resolution_clock::now();

  // Check if some nodes have been added to this System
  if (m_nodes.empty()) {
    assert(0 && "Error. Calling performance_test::System::spin when no nodes have been added.");
  }

  if (m_events_logger != nullptr) {
    m_events_logger->set_start_time(m_start_time);
  }

  if (wait_for_discovery) {
    // wait until PDP and EDP are finished before starting
    // log events when each is completed
    RCLCPP_INFO(rclcpp::get_logger("ros2-performance"), "Waiting for discovery");
    this->wait_discovery();
  }

  RCLCPP_INFO(rclcpp::get_logger("ros2-performance"), "Starting to spin");
  for (const auto & pair : m_executors_map) {
    auto & name = pair.second.name;
    auto & executor = pair.second.executor;

    // Spin each executor in a separate thread
    auto thread = create_spin_thread(executor);

    pthread_setname_np(thread->native_handle(), name.c_str());

    m_threads.push_back(std::move(thread));
  }

  // let the nodes spin for the specified amount of time
  performance_test::sleep_task(m_experiment_duration);
  // If spin type is waiting on a promise, fulfill the promise
  if (m_spin_type == SpinType::SPIN_FUTURE_COMPLETE) {
    m_promise.set_value();
  }

  // after the timer, stop all the spin functions
  for (const auto & pair : m_executors_map) {
    auto & executor = pair.second.executor;
    executor->cancel();
  }
}

std::unique_ptr<std::thread> System::create_spin_thread(rclcpp::Executor::SharedPtr executor)
{
  std::unique_ptr<std::thread> thread;

  switch (m_spin_type) {
    case SpinType::SPIN:
      thread = std::make_unique<std::thread>(
        [executor]() {
          executor->spin();
        });
      break;
    case SpinType::SPIN_SOME:
      thread = std::make_unique<std::thread>(
        [executor]() {
          while (rclcpp::ok()) {
            executor->spin_some();
          }
        });
      break;
    case SpinType::SPIN_FUTURE_COMPLETE:
      thread = std::make_unique<std::thread>(
        [executor, this]() {
          executor->spin_until_future_complete(m_promise.get_future());
        });
      break;
  }

  return thread;
}

void System::save_latency_all_stats(
  const std::string & results_folder,
  bool include_services) const
{
  if (results_folder.empty()) {
    std::cout << "[SystemLatencyLogger]: Error. Provided an empty filename." << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  auto filename = results_folder + "/latency_all.txt";

  std::ofstream out_file;
  out_file.open(filename);

  if (!out_file.is_open()) {
    std::cout << "[SystemLatencyLogger]: Error. Could not open file " << filename << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  this->log_latency_all_stats(out_file, include_services);
}

void System::save_latency_total_stats(
  const std::string & filename,
  bool include_services) const
{
  if (filename.empty()) {
    std::cout << "[SystemLatencyLogger]: Error. Provided an empty filename." << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  std::ofstream out_file;
  out_file.open(filename);

  if (!out_file.is_open()) {
    std::cout << "[SystemLatencyLogger]: Error. Could not open file " << filename << std::endl;
    std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
    return;
  }

  this->log_latency_total_stats(out_file, include_services);
}

void System::log_latency_all_stats(
  std::ostream & stream,
  bool include_services) const
{
  std::vector<performance_metrics::Tracker> subs;

  for (const auto & n : m_nodes) {
    auto trackers = n->sub_trackers();
    subs.insert(subs.end(), trackers.begin(), trackers.end());
  }

  performance_metrics::log_trackers_latency_all_stats(
    stream,
    subs,
    m_csv_out,
    "Subscriptions stats:");

  if (include_services) {
    std::vector<performance_metrics::Tracker> clients;
    std::vector<performance_metrics::Tracker> services;
    std::vector<performance_metrics::Tracker> action_clients;
    std::vector<performance_metrics::Tracker> action_servers;

    for (const auto & n : m_nodes) {
      auto service_trackers = n->service_trackers();
      auto client_trackers = n->client_trackers();
      auto action_client_trackers = n->action_client_trackers();
      auto action_server_trackers = n->action_server_trackers();
      clients.insert(clients.end(), client_trackers.begin(), client_trackers.end());
      services.insert(services.end(), service_trackers.begin(), service_trackers.end());
      action_clients.insert(action_clients.end(), action_client_trackers.begin(), action_client_trackers.end());
      action_servers.insert(action_servers.end(), action_server_trackers.begin(), action_server_trackers.end());
    }

    performance_metrics::log_trackers_latency_all_stats(
      stream,
      clients,
      m_csv_out,
      "Clients stats:");

    performance_metrics::log_trackers_latency_all_stats(
      stream,
      services,
      m_csv_out,
      "Services stats:");

    performance_metrics::log_trackers_latency_all_stats(
      stream,
      action_clients,
      m_csv_out,
      "Action Clients stats:");

    performance_metrics::log_trackers_latency_all_stats(
      stream,
      action_servers,
      m_csv_out,
      "Action Servers stats:");
  }

  std::vector<performance_metrics::Tracker> publishers;

  for (const auto & n : m_nodes) {
    auto trackers = n->pub_trackers();
    publishers.insert(publishers.end(), trackers.begin(), trackers.end());
  }

  performance_metrics::log_trackers_latency_all_stats(
    stream,
    publishers,
    m_csv_out,
    "Publishers stats:");
}

void System::log_latency_total_stats(
  std::ostream & stream,
  bool include_services) const
{
  std::vector<performance_metrics::Tracker> all_trackers;
  for (const auto & n : m_nodes) {
    auto sub_trackers = n->sub_trackers();
    all_trackers.insert(all_trackers.end(), sub_trackers.begin(), sub_trackers.end());
    if (include_services) {
      auto services_trackers = n->service_trackers();
      all_trackers.insert(all_trackers.end(), services_trackers.begin(), services_trackers.end());
      auto client_trackers = n->client_trackers();
      all_trackers.insert(all_trackers.end(), client_trackers.begin(), client_trackers.end());
      auto action_client_trackers = n->action_client_trackers();
      all_trackers.insert(all_trackers.end(), action_client_trackers.begin(), action_client_trackers.end());
      auto action_server_trackers = n->action_server_trackers();
      all_trackers.insert(all_trackers.end(), action_server_trackers.begin(), action_server_trackers.end());
    }
  }
  performance_metrics::log_trackers_latency_total_stats(stream, all_trackers, m_csv_out);
}

void System::print_aggregate_stats(
  const std::vector<std::string> & topology_json_list,
  const std::string & results_folder_path) const
{
  uint64_t total_received = 0;
  uint64_t total_lost = 0;
  uint64_t total_late = 0;
  uint64_t total_too_late = 0;
  uint64_t total_latency = 0;

  for (const auto & json : topology_json_list) {
    try {
      std::string basename = json.substr(json.find_last_of("/") + 1, json.length());
      std::string filename = basename.substr(0, basename.length() - 5) + "_log/latency_total.txt";
      std::string line;
      if (results_folder_path != "") {
        filename = results_folder_path + "/latency_total.txt";
      }

      std::ifstream log_file(filename);

      if (log_file.is_open()) {
        getline(log_file, line);
        // The second line contains the data to parse
        getline(log_file, line);

        total_received += parse_line(line, m_csv_out);
        total_latency += parse_line(line, m_csv_out);
        total_late += parse_line(line, m_csv_out);
        parse_line(line, m_csv_out);
        total_too_late += parse_line(line, m_csv_out);
        parse_line(line, m_csv_out);
        total_lost += parse_line(line, m_csv_out);
        log_file.close();
      } else {
        std::cout << "[SystemLatencyLogger]: Error. Could not open file " << filename << std::endl;
      }
    } catch (const std::out_of_range& e) {
      std::cerr << "Out of range error: " << e.what() << " while processing json: " << json << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Exception: " << e.what() << " while processing json: " << json << std::endl;
    }
  }

  double average_latency = std::round(total_latency / topology_json_list.size());

  performance_metrics::log_total_stats(
    total_received, total_lost, total_late, total_too_late,
    average_latency, std::cout, m_csv_out);
}

void System::wait_discovery()
{
  // period at which PDP and EDP are checked
  auto period = std::chrono::milliseconds(30);
  // maximum discovery time, after which the experiment is shut down
  auto max_discovery_time = std::chrono::seconds(30);

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

  auto get_intersection_size = [ = ](std::vector<std::string> A, std::vector<std::string> B) {
      // returns how many values are present in both A and B
      std::sort(A.begin(), A.end());
      std::sort(B.begin(), B.end());
      std::vector<std::string> v_intersection;
      std::set_intersection(
        A.begin(), A.end(),
        B.begin(), B.end(),
        std::back_inserter(v_intersection));
      return v_intersection.size();
    };

  // create a vector with all the names of the nodes to be discovered
  std::vector<std::string> reference_names;
  for (const auto & n : m_nodes) {
    std::string node_name = n->get_node_base()->get_fully_qualified_name();
    reference_names.push_back(node_name);
  }

  // count the total number of nodes
  size_t num_nodes = m_nodes.size();

  bool pdp_ok = false;
  while (!pdp_ok) {
    for (const auto & n : m_nodes) {
      // we use the intersection to avoid counting nodes discovered from other processes
      size_t discovered_participants =
        get_intersection_size(n->get_node_graph()->get_node_names(), reference_names);
      pdp_ok = (discovered_participants == num_nodes);
      if (!pdp_ok) {break;}
    }

    if (pdp_ok) {break;}

    // check if maximum discovery time exceeded
    auto t = std::chrono::high_resolution_clock::now();
    auto duration = t - pdp_start_time;
    if (duration > max_pdp_time) {
      assert(0 && "[discovery] PDP took more than maximum discovery time");
    }

    rate.sleep();
  }

  if (m_events_logger != nullptr) {
    // Create an event for PDP completed
    performance_metrics::EventsLogger::Event pdp_ev;
    pdp_ev.caller_name = "SYSTEM";
    pdp_ev.code = performance_metrics::EventsLogger::EventCode::discovery;
    pdp_ev.description = "[discovery] PDP completed";
    m_events_logger->write_event(pdp_ev);
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
  for (const auto & n : m_nodes) {
    auto trackers = n->sub_trackers();
    for (const auto & tracker : trackers) {
      subs_per_topic[tracker.get_entity_name()] += 1;
    }
  }

  // The EDP should also take into account if subscriptions have been matched with publishers
  // This is needed in case of processes with only subscriptions
  bool edp_ok = false;
  while (!edp_ok) {
    for (const auto & n : m_nodes) {
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
    auto duration = t - edp_start_time;
    if (duration > max_edp_time) {
      assert(0 && "[discovery] EDP took more than maximum discovery time");
    }

    rate.sleep();
  }

  if (m_events_logger != nullptr) {
    // Create an event for EDP completed
    performance_metrics::EventsLogger::Event edp_ev;
    edp_ev.caller_name = "SYSTEM";
    edp_ev.code = performance_metrics::EventsLogger::EventCode::discovery;
    edp_ev.description = "[discovery] EDP completed";
    m_events_logger->write_event(edp_ev);
  }
}

}  // namespace performance_test
