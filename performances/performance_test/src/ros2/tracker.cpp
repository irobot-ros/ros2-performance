/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <sstream>

#include "performance_test/ros2/tracker.hpp"


void performance_test::Tracker::scan(
    const performance_test_msgs::msg::PerformanceHeader& header,
    const rclcpp::Time& now,
    std::shared_ptr<EventsLogger> elog)
{
    // Compute latency
    rclcpp::Time stamp(header.stamp.sec, header.stamp.nanosec, RCL_ROS_TIME);
    auto lat = std::chrono::nanoseconds((now - stamp).nanoseconds());
    unsigned long lat_us = lat.count() / 1000;
    // store the last latency to be read from node
    _last_latency = lat_us;

    bool late = false;
    bool too_late = false;

    if (_tracking_options.is_enabled){
	
        auto it = _tracking_number_count_map.find(header.node_id);
        // If this is first message received for the node store some info about it
        if (it == _tracking_number_count_map.end()) {
            _size.add_sample(header.size);
            _frequency.add_sample(header.frequency);
            it = _tracking_number_count_map.insert(it, {header.node_id, header.tracking_number});
        }
	
        // Check if we received the correct message. The assumption here is
        // that the messages arrive in chronological order
        if (header.tracking_number == it->second) {
            it->second++;
        } else if (header.tracking_number > it->second) {
            // We missed some mesages...
            long unsigned int n_lost = header.tracking_number - it->second;
            _lost_messages += n_lost;
            it->second = header.tracking_number + 1;
            // Log the event
            if (elog != nullptr){
                EventsLogger::Event ev;
                std::stringstream description;
                ev.caller_name = _topic_srv_name + "->" + _node_name;
                ev.code = EventsLogger::EventCode::lost_messages;

                if(n_lost == 1) {
                    description << "msg " << header.tracking_number - 1 << " lost.";
                } else {
                    description << "msgs " << header.tracking_number - 1 << " to " << header.tracking_number - 1 + n_lost << " lost.";
                }
                ev.description = description.str();
                elog->write_event(ev);
            }
        } else {
            std::cerr << "warning: order of message is not sequential! Is node_name unique for each publisher?" << std::endl;
        }

        // Check if the message latency qualifies the message as a lost or late message.
        const int  period_us = 1000000 / header.frequency;
        const unsigned int latency_late_threshold_us = std::min(_tracking_options.late_absolute_us,
                                                                _tracking_options.late_percentage * period_us / 100);
        const unsigned int latency_too_late_threshold_us = std::min(_tracking_options.too_late_absolute_us,
                                                                _tracking_options.too_late_percentage * period_us / 100);

        too_late = lat_us > latency_too_late_threshold_us;
        late = lat_us > latency_late_threshold_us && !too_late;

        if(late) {
            if (elog != nullptr){
                // Create a description for the event
                std::stringstream description;
                description << "msg "<< header.tracking_number << " late. "
                << lat_us << "us > "<< latency_late_threshold_us << "us";

                EventsLogger::Event ev;
                ev.caller_name = _topic_srv_name + "->" + _node_name;
                ev.code = EventsLogger::EventCode::late_message;
                ev.description = description.str();

                elog->write_event(ev);
            }
            _late_messages++;
        }

        if(too_late) {
            if (elog != nullptr){
                // Create a descrption for the event
                std::stringstream description;
                description << "msg "<< header.tracking_number << " too late. "
                << lat_us << "us > "<< latency_too_late_threshold_us << "us";

                EventsLogger::Event ev;
                ev.caller_name = _topic_srv_name + "->" + _node_name;
                ev.code = EventsLogger::EventCode::too_late_message;
                ev.description = description.str();

                elog->write_event(ev);
            }
            _too_late_messages++;
        }
    }

    if(!too_late) {
        // Compute statistics with new sample. Don't add to this the msgs
        // that arrived too late.
        _stat.add_sample(lat_us);
    }

    _received_messages++;
}
