#ifndef __TEMPLATE_FACTORY_HPP__
#define __TEMPLATE_FACTORY_HPP__

#include <string>
#include <vector>

#include "performance_test/ros2/multi_node.hpp"

class TemplateFactory {

    public:

		TemplateFactory() : _ros2_namespace("") {}
		TemplateFactory(std::string ros2_namespace) : _ros2_namespace(ros2_namespace) {}
        /**
         * create a single node
         */
        std::shared_ptr<MultiNode> make_templated(const std::string& msg_type, int id);

        std::shared_ptr<MultiNode> make_templated(const std::string& msg_type, std::string name);
        /**
         * create several nodes at the same time
         */
        std::vector<std::shared_ptr<MultiNode>> create_subscribers(int start_id, int end_id, int n_publishers, std::string msg_type, bool verbose = false, rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        std::vector<std::shared_ptr<MultiNode>> create_publishers(int start_id, int end_id, std::string msg_type, bool verbose = false, rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        std::vector<std::shared_ptr<MultiNode>> create_clients(int start_id, int end_id, int n_services, std::string msg_type, bool verbose = false, rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);

        std::vector<std::shared_ptr<MultiNode>> create_servers(int start_id, int end_id, std::string msg_type, bool verbose = false, rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default);
        /**
         * start task for some nodes
         */
        void start_subscribers(std::vector<std::shared_ptr<MultiNode>> nodes, float frequency, rclcpp::executor::Executor::SharedPtr executor);

        void start_publishers(std::vector<std::shared_ptr<MultiNode>> nodes, float frequency, int task_duration_sec, int msg_size, rclcpp::executor::Executor::SharedPtr executor);

        void start_clients(std::vector<std::shared_ptr<MultiNode>> nodes, float frequency, int task_duration_sec, rclcpp::executor::Executor::SharedPtr executor);

        void start_servers(std::vector<std::shared_ptr<MultiNode>> nodes, float frequency, rclcpp::executor::Executor::SharedPtr executor);

    private:

        std::string _ros2_namespace;
};

#endif
