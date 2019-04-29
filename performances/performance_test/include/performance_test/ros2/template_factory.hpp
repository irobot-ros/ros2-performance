#pragma once

#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "performance_test/ros2/node.hpp"

namespace performance_test {

class TemplateFactory {

    public:

		TemplateFactory(std::string ros2_namespace = "") : _ros2_namespace(ros2_namespace) {}

        /**
         * Helper functions for creating several nodes at the same time.
         * These nodes will have names as "node_X", "node_Y"
         * where X, Y, etc, are all the numbers spanning from `start_id` to `end_id`.
         * The topic or service type is obtained parsing the msg_type std::string
         */

        std::vector<std::shared_ptr<Node>> create_subscriber_nodes(
            int start_id,
            int end_id,
            int n_publishers,
            std::string msg_type,
            Tracker::TrackingOptions tracking_options,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default,
            bool use_ipc = true,
            bool verbose = false);

        std::vector<std::shared_ptr<Node>> create_periodic_publisher_nodes(
            int start_id,
            int end_id,
            float frequency,
            std::string msg_type,
            size_t msg_size = 0,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default,
            bool use_ipc = true,
            bool verbose = false);

        std::vector<std::shared_ptr<Node>> create_periodic_client_nodes(
            int start_id,
            int end_id,
            int n_services,
            float frequency,
            std::string msg_type,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default,
            bool use_ipc = true,
            bool verbose = false);

        std::vector<std::shared_ptr<Node>> create_server_nodes(
            int start_id,
            int end_id,
            std::string msg_type,
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default,
            bool use_ipc = true,
            bool verbose = false);

        static size_t get_msg_size(std::string msg_type, size_t msg_size = 0);

        /**
         * Helper functions that, given a node and a std::string describing the msg_type,
         * create the publisher/subscriber/client/server accordingly
         */

        void add_subscriber_from_strings(
            std::shared_ptr<Node> n,
            std::string msg_type,
            std::string topic_name,
            Tracker::TrackingOptions tracking_options,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default);

        void add_periodic_publisher_from_strings(
            std::shared_ptr<Node> n,
            std::string msg_type,
            std::string topic_name,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
            std::chrono::milliseconds period_ms = std::chrono::milliseconds(10),
            size_t msg_size = 0);

        void add_periodic_client_from_strings(
            std::shared_ptr<Node> n,
            std::string srv_type,
            std::string service_name,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default,
            std::chrono::milliseconds period_ms = std::chrono::milliseconds(10));

        void add_server_from_strings(
            std::shared_ptr<Node> n,
            std::string srv_type,
            std::string service_name,
            rmw_qos_profile_t qos_profile = rmw_qos_profile_default);


        /**
         * Helper functions that, given a given a json file describing a system,
         * create the nodes accordingly
         */


        std::vector<std::shared_ptr<Node>> parse_nodes_from_json(std::string json_path);

        std::shared_ptr<Node> create_node_from_json(const nlohmann::json node_json);

        void add_periodic_publisher_from_json(std::shared_ptr<Node> node, const nlohmann::json pub_json);

        void add_subscriber_from_json(std::shared_ptr<Node> node, const nlohmann::json sub_json);

        void add_periodic_client_from_json(std::shared_ptr<Node> node, const nlohmann::json client_json);

        void add_server_from_json(std::shared_ptr<Node> node, const nlohmann::json server_json);

    private:

        std::string _ros2_namespace;
};

}