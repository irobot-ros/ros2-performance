#ifndef __MULTI_NODE_HPP__
#define __MULTI_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "performance_test/ros2/node_stats.hpp"

/**
 * abstract class derived from rclcpp::Node
 * it's used to declare all the APIs that a user can call on the templated, derived, class TemplatedMultiNode
 */
struct MultiNode : public rclcpp::Node {

    MultiNode(std::string name, std::string ros2_namespace): Node(name, ros2_namespace){}

    virtual ~MultiNode() = default;

    // external apis for creating communication interfaces
    // it is possible to use a numeric ID or a string for the names of the topics
    virtual void add_publisher(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default) = 0;
    virtual void add_publisher(std::string name, rmw_qos_profile_t qos_profile = rmw_qos_profile_default) = 0;
    virtual void add_subscriber(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default) = 0;
    virtual void add_subscriber(std::string name, rmw_qos_profile_t qos_profile = rmw_qos_profile_default) = 0;
    virtual void add_service(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default) = 0;
    virtual void add_client(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default) = 0;
    virtual void add_timer(float frequency, std::function< void() > callback) = 0;

    // basic node tasks
    // they run for the specified amount of time or until rclcpp::shutdown is called
    virtual void simple_spin_task(float frequency) = 0;
    virtual void simple_publisher_task(float frequency, int task_duration_sec, int size = 0) = 0;
    virtual void simple_client_task(float frequency, int task_duration_sec) = 0;

    // methods iteratively called by the node tasks previously defined
    // they can be also used as timer callbacks
    virtual void publish_messages(float frequency, int task_duration_sec, int size = 0) = 0;
    virtual void send_requests(float frequency, int task_duration_sec) = 0;

    // node identifiers
    int _id;
    std::string _name;
    int get_id(){ return _id;}

    // get size of templated message
    int get_message_size()
    {
        return _msg_size;
    }

    // this value is set when publishing/receiving messages.
    // TODO: find better solution as it may be set multiple times with different values
    int _msg_size;

    // record statistics
    NodeStats stats;

};




#endif //__MULTI_NODE_HPP__
