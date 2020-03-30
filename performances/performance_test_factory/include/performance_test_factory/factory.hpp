/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "performance_test/ros2/node.hpp"

namespace performance_test {

class TemplateFactory {

    public:

        TemplateFactory(
            bool use_ipc = true,
            bool use_ros_params = true,
            bool verbose_mode = false,
            std::string ros2_namespace = "") :
                _use_ipc(use_ipc),
                _use_ros_params(use_ros_params),
                _verbose_mode(verbose_mode),
                _ros2_namespace(ros2_namespace)
        {}

        /**
         * Helper functions for creating several nodes at the same time.
         * These nodes will have names as "node_X", "node_Y"
         * where X, Y, etc, are all the numbers spanning from `start_id` to `end_id`.
         * The topic or service type is obtained parsing the msg_type std::string
         */

        std::shared_ptr<Node> create_node(
            std::string name,
            bool use_ipc = true,
            bool use_ros_params = true,
            bool verbose = false,
            std::string ros2_namespace = "",
            int executor_id = 0);

        std::vector<std::shared_ptr<Node>> create_subscriber_nodes(
            int start_id,
            int end_id,
            int n_publishers,
            std::string msg_type,
            msg_pass_by_t msg_pass_by,
            Tracker::TrackingOptions tracking_options = Tracker::TrackingOptions(),
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        std::vector<std::shared_ptr<Node>> create_periodic_publisher_nodes(
            int start_id,
            int end_id,
            float frequency,
            std::string msg_type,
            msg_pass_by_t msg_pass_by,
            size_t msg_size = 0,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        std::vector<std::shared_ptr<Node>> create_periodic_client_nodes(
            int start_id,
            int end_id,
            int n_services,
            float frequency,
            std::string msg_type,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        std::vector<std::shared_ptr<Node>> create_server_nodes(
            int start_id,
            int end_id,
            std::string msg_type,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        /**
         * Helper functions that, given a node and a std::string describing the msg_type,
         * create the publisher/subscriber/client/server accordingly
         */

        void add_subscriber_from_strings(
            std::shared_ptr<Node> n,
            std::string msg_type,
            std::string topic_name,
            Tracker::TrackingOptions tracking_options,
            msg_pass_by_t msg_pass_by = PASS_BY_SHARED_PTR,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default);

        void add_periodic_publisher_from_strings(
            std::shared_ptr<Node> n,
            std::string msg_type,
            std::string topic_name,
            msg_pass_by_t msg_pass_by = PASS_BY_UNIQUE_PTR,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
            std::chrono::microseconds period_us = std::chrono::microseconds(10000),
            size_t msg_size = 0);

        void add_periodic_client_from_strings(
            std::shared_ptr<Node> n,
            std::string srv_type,
            std::string service_name,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
            std::chrono::microseconds period_us = std::chrono::microseconds(10000));

        void add_server_from_strings(
            std::shared_ptr<Node> n,
            std::string srv_type,
            std::string service_name,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default);


        /**
         * Helper function that, given a given a json file describing a system,
         * parses it and creates the nodes accordingly
         */

        std::vector<std::shared_ptr<Node>> parse_topology_from_json(
            std::string json_path,
            Tracker::TrackingOptions tracking_options = Tracker::TrackingOptions());

    private:

        std::shared_ptr<Node> create_node_from_json(const nlohmann::json node_json, std::string suffix = "");

        void create_node_entities_from_json(std::shared_ptr<Node> node, const nlohmann::json node_json, Tracker::TrackingOptions tracking_options = Tracker::TrackingOptions());

        void add_periodic_publisher_from_json(std::shared_ptr<Node> node, const nlohmann::json pub_json);

        void add_subscriber_from_json(std::shared_ptr<Node> node, const nlohmann::json sub_json, Tracker::TrackingOptions tracking_options);

        void add_periodic_client_from_json(std::shared_ptr<Node> node, const nlohmann::json client_json);

        void add_server_from_json(std::shared_ptr<Node> node, const nlohmann::json server_json);

        rmw_qos_profile_t get_qos_from_json(const nlohmann::json entity_json);

        msg_pass_by_t get_msg_pass_by_from_json(const nlohmann::json entity_json, msg_pass_by_t default_value);

        bool _use_ipc;
        bool _use_ros_params;
        bool _verbose_mode;
        std::string _ros2_namespace;

};

}