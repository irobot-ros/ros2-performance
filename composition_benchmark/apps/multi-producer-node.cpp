/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include "irobot_interfaces_plugin/msg/stamped10b.hpp"
#include "irobot_interfaces_plugin/msg/stamped100kb.hpp"
#include "composition_benchmark/base_node.hpp"
#include "composition_benchmark/helpers/helper_options.hpp"
#include "composition_benchmark/helpers/helper_types.hpp"
#include "performance_test/system.hpp"
#include "performance_test/utils/node_options.hpp"
#include "performance_metrics/resource_usage_logger.hpp"

class MultiProducerNode : public performance_test::PerformanceNode<rclcpp::Node>
{
public:
    MultiProducerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : performance_test::PerformanceNode<rclcpp::Node>(
        "MultiProducerNode",
        "",
        options)
    {
        int topics_number = this->declare_parameter<int>("topics_number", 1);
        std::vector<std::function<void ()>> publish_functions;
        for (int i = 0; i < topics_number; i++) {

            using Msg = irobot_interfaces_plugin::msg::Stamped10b;

            std::string topic_name = "topic_" + std::to_string(i);
            this->add_publisher<Msg>(topic_name, rclcpp::SensorDataQoS());

            auto publish_func = std::bind(
                &MultiProducerNode::publish_msg<Msg>,
                this,
                topic_name,
                performance_test::msg_pass_by_t::PASS_BY_UNIQUE_PTR,
                0,
                std::chrono::milliseconds(10));
            publish_functions.push_back(publish_func);
        }

        this->add_timer(std::chrono::milliseconds(10), [this, publish_functions](){
            for (auto & func : publish_functions) {
                func();
            }
        });

        this->add_periodic_publisher<irobot_interfaces_plugin::msg::Stamped100kb>(
            "slow_topic",
            std::chrono::milliseconds(111),
            performance_test::msg_pass_by_t::PASS_BY_UNIQUE_PTR,
            rclcpp::SensorDataQoS());
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  performance_metrics::ResourceUsageLogger ru_logger("/data/log");
  ru_logger.start(std::chrono::milliseconds(500));

  auto nodes = std::make_shared<MultiProducerNode>();
  auto system = std::make_unique<performance_test::System>(
    performance_test::ExecutorType::SINGLE_THREADED_EXECUTOR,
    performance_test::SpinType::SPIN);

  system->add_node(nodes);
  system->spin(std::chrono::seconds(20), false);

  ru_logger.stop();

  rclcpp::shutdown();
}
