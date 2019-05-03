/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <fstream>
#include <map>

#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/names_utilities.hpp"


performance_test::System::System(int executor_id)
{
    //TODO here implement also multi thread executor, or maybe allow user to pass its custom executor
    if (executor_id == 1){
        _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    else{
        _executor = nullptr;
    }

}


void performance_test::System::enable_events_logger(std::string events_logger_path)
{
    _events_logger = std::make_shared<EventsLogger>(events_logger_path);
}


void performance_test::System::add_node(std::vector<std::shared_ptr<Node>> nodes)
{
    for (auto node : nodes){
        this->add_node(node);
    }
}


void performance_test::System::add_node(std::shared_ptr<Node> node)
{
    if (_events_logger != nullptr){
        node->set_events_logger(_events_logger);
    }

    _nodes.push_back(node);
}


// TODO: Is it worth to spin each node in a different thread?
void performance_test::System::spin(int duration_sec, bool wait_for_discovery)
{
    _experiment_duration_sec = duration_sec;

    // Store the instant when the experiment started
    _start_time = std::chrono::high_resolution_clock::now();

    if (_events_logger != nullptr){
        _events_logger->set_start_time(_start_time);
    }

    if (wait_for_discovery){
        // wait until PDP and EDP are finished before starting
        // log events when each is completed
        this->wait_discovery();
    }

    if (_executor != nullptr){
        // a main executor has been defined, so add all nodes to it
        for (const auto& n : _nodes){
            _executor->add_node(n);
        }
        // create a separate thread for the executor to spin
        std::thread thread([&](){
            _executor->spin();
        });
        thread.detach();
    }
    else{
        // Create a different executor for each node
        for (const auto& n : _nodes){
            auto ex = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            ex->add_node(n);
            // Spin each executor in a different thread
            std::thread a([=]() { ex->spin(); });
            a.detach();
            _executors_vec.push_back(ex);
        }
    }

    // let the nodes spin for the specified amount of time
    std::this_thread::sleep_for(std::chrono::seconds(_experiment_duration_sec));

    // after the timer, stop all the spin functions
    if (_executor != nullptr){
        _executor->cancel();
    }
    else{
        for (auto& ex : _executors_vec){
            ex->cancel();
        }
    }
}


void performance_test::System::wait_discovery()
{
    // period at which PDP and EDP are checked
    rclcpp::WallRate rate(20ms);
    // maximum discovery time, after which the experiment is shut down
    std::chrono::milliseconds max_discovery_ms = 60s;

    /**
     * Check PDP
     */

    // count the total number of nodes
    size_t num_nodes = _nodes.size();

    bool pdp_ok = false;
    while (!pdp_ok){
        for (const auto& n : _nodes){
            // dividing by 2 because half of the entries are empty
            // TODO check if it is always true
            auto discovered_participants = n->get_node_names().size()/2;

            pdp_ok = (discovered_participants == num_nodes);

            if (!pdp_ok) break;
        }

        if (pdp_ok) break;

        // check if maximum discovery time exceeded
        auto t = std::chrono::high_resolution_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(t - _start_time - max_discovery_ms).count();
        if (duration > 0){
            assert(0 && "[discovery] PDP took more than maximum discovery time");
        }

        rate.sleep();
    }

    if (_events_logger != nullptr){
        // Create an event for PDP completed
        EventsLogger::Event pdp_ev;
        pdp_ev.caller_name = "SYSTEM";
        pdp_ev.code = EventsLogger::EventCode::discovery;
        pdp_ev.description = "[discovery] PDP completed";
        _events_logger->write_event(pdp_ev);
    }

    /**
     * Check EDP
     */

    // count the number of subscribers for each topic
    std::map<std::string, int> subs_per_topic;
    for (const auto& n : _nodes){
        auto trackers = n->all_trackers();
        for (const auto& tracker : *trackers){
            subs_per_topic[tracker.first] += 1;
        }
    }

    bool edp_ok = false;
    while (!edp_ok){
        for (const auto& n : _nodes){
            for (const auto& pub_tracker : n->_pubs){
                std::string topic_name = pub_tracker.first;
                int discovered_endpoints = n->count_subscribers(topic_name);
                edp_ok = (discovered_endpoints == subs_per_topic[topic_name]);

                if (!edp_ok) break;
            }
        }

        if (edp_ok) break;

        // check if maximum discovery time exceeded
        auto t = std::chrono::high_resolution_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(t - _start_time - max_discovery_ms).count();
        if (duration > 0){
            assert(0 && "[discovery] EDP took more than maximum discovery time");
        }

        rate.sleep();
    }

    if (_events_logger != nullptr){
        // Create an event for EDP completed
        EventsLogger::Event edp_ev;
        edp_ev.caller_name = "SYSTEM";
        edp_ev.code = EventsLogger::EventCode::discovery;
        edp_ev.description = "[discovery] EDP completed";
        _events_logger->write_event(edp_ev);
    }
}


void performance_test::System::save_latency_all_stats(std::string filename)
{

    if (filename.empty()){
        std::cout<<"[SystemLatencyLogger]: Error. Provided an empty filename."<<std::endl;
        std::cout<<"[SystemLatencyLogger]: Not logging."<<std::endl;
        return;
    }

    std::ofstream out_file;
    out_file.open(filename);

    if(!out_file.is_open()) {
        std::cout << "[SystemLatencyLogger]: Error. Could not open file "<< filename<< std::endl;
        std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
        return;
    }

    this->log_latency_all_stats(out_file);
}

void performance_test::System::save_latency_total_stats(std::string filename)
{

    if (filename.empty()){
        std::cout<<"[SystemLatencyLogger]: Error. Provided an empty filename."<<std::endl;
        std::cout<<"[SystemLatencyLogger]: Not logging."<<std::endl;
        return;
    }

    std::ofstream out_file;
    out_file.open(filename);

    if(!out_file.is_open()) {
        std::cout << "[SystemLatencyLogger]: Error. Could not open file "<< filename<< std::endl;
        std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
        return;
    }

    this->log_latency_total_stats(out_file);
}


void performance_test::System::print_latency_all_stats()
{
    this->log_latency_all_stats(std::cout);
}

void performance_test::System::print_latency_total_stats()
{
    this->log_latency_total_stats(std::cout);
}


void performance_test::System::log_latency_all_stats(std::ostream& stream)
{
    const char separator = ' ';
    const int name_width = 12;
    const int num_width = 10;

    stream << std::left << std::setw(name_width) << std::setfill(separator) << "node";
    stream << std::left << std::setw(name_width) << std::setfill(separator) << "topic";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "size[b]";
    stream << std::left << std::setw(num_width+4) << std::setfill(separator) << "received[#]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "late[#]";
    stream << std::left << std::setw(num_width+2) << std::setfill(separator) << "too_late[#]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "lost[#]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "mean[us]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "sd[us]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "min[us]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "max[us]";
    stream << std::left << std::setw(num_width+2) << std::setfill(separator) << "freq[hz]";
    stream << std::left << std::setw(num_width) << std::setfill(separator) << "duration[s]";

    stream << std::endl;

    unsigned long int total_received = 0;
    unsigned long int total_lost = 0;
    unsigned long int total_late = 0;
    unsigned long int total_too_late = 0;

    // Print all
    for (const auto& n : _nodes)
    {
        auto trackers = n->all_trackers();
        for(const auto& tracker : *trackers)
        {
            total_received += tracker.second.received();
            total_lost += tracker.second.lost();
            total_late += tracker.second.late();
            total_too_late += tracker.second.too_late();

            stream << std::left << std::setw(name_width) << std::setfill(separator) << n->get_name();
            stream << std::left << std::setw(name_width) << std::setfill(separator) << tracker.first;
            stream << std::left << std::setw(num_width) << std::setfill(separator) << tracker.second.size();
            stream << std::left << std::setw(num_width+4) << std::setfill(separator) << tracker.second.received();
            stream << std::left << std::setw(num_width) << std::setfill(separator) << tracker.second.late();
            stream << std::left << std::setw(num_width+2) << std::setfill(separator) << tracker.second.too_late();
            stream << std::left << std::setw(num_width) << std::setfill(separator) << tracker.second.lost();
            stream << std::left << std::setw(num_width) << std::setfill(separator) << std::round(tracker.second.stat().mean());
            stream << std::left << std::setw(num_width) << std::setfill(separator) << std::round(tracker.second.stat().stddev());
            stream << std::left << std::setw(num_width) << std::setfill(separator) << std::round(tracker.second.stat().min());
            stream << std::left << std::setw(num_width) << std::setfill(separator) << std::round(tracker.second.stat().max());
            stream << std::left << std::setw(num_width+2) << std::setfill(separator) << std::round(tracker.second.frequency());
            stream << std::left << std::setw(num_width) << std::setfill(separator) << _experiment_duration_sec;
            stream << std::endl;
        }
    }
}

void performance_test::System::log_latency_total_stats(std::ostream& stream)
{
    const char separator = ' ';
    const int name_width = 12;
    const int num_width = 10;

    unsigned long int total_received = 0;
    unsigned long int total_lost = 0;
    unsigned long int total_late = 0;
    unsigned long int total_too_late = 0;

    // Print all
    for (const auto& n : _nodes)
    {
        auto trackers = n->all_trackers();
        for(const auto& tracker : *trackers)
        {
            total_received += tracker.second.received();
            total_lost += tracker.second.lost();
            total_late += tracker.second.late();
            total_too_late += tracker.second.too_late();
        }
    }

    double total_lost_percentage = (double)total_lost / (total_received + total_lost) * 100;
    double total_late_percentage = (double)total_late / total_received * 100;
    double total_too_late_percentage = (double)total_too_late / total_received * 100;

    stream << std::left << std::setw(name_width+4) << std::setfill(separator) << "received[#]";
    stream << std::left << std::setw(name_width) << std::setfill(separator) << "late[#]";
    stream << std::left << std::setw(name_width) << std::setfill(separator) << "late[%]";
    stream << std::left << std::setw(name_width+4) << std::setfill(separator) << "too_late[#]";
    stream << std::left << std::setw(name_width+4) << std::setfill(separator) << "too_late[%]";
    stream << std::left << std::setw(name_width) << std::setfill(separator) << "lost[#]";
    stream << std::left << std::setw(name_width) << std::setfill(separator) << "lost[%]" << std::endl;

    stream << std::left << std::setw(name_width+4) << std::setfill(separator) << total_received;
    stream << std::left << std::setw(name_width) << std::setfill(separator) << total_late ;
    stream << std::left << std::setw(name_width) << std::setfill(separator) << std::setprecision(4) << total_late_percentage;
    stream << std::left << std::setw(name_width+4) << std::setfill(separator) << total_too_late ;
    stream << std::left << std::setw(name_width+4) << std::setfill(separator) << std::setprecision(4) << total_too_late_percentage;
    stream << std::left << std::setw(name_width) << std::setfill(separator) << total_lost;
    stream << std::left << std::setw(name_width) << std::setfill(separator) << std::setprecision(4) << total_lost_percentage << std::endl;
}