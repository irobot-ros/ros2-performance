#ifndef __TEMPLATED_MULTI_NODE_HPP__
#define __TEMPLATED_MULTI_NODE_HPP__

#include "performance_test/ros2/names_utilities.hpp"
#include "performance_test/ros2/multi_node.hpp"

#include "performance_test/srv/get_header.hpp"
#include "performance_test/msg/stamped_vector.hpp" //This include is needed for std::enable_if std::is_same

#include <chrono>
#include <map>

/**
 * templated class providing implementations for MultiNode virtual methods
 */
template <typename MsgType>
struct TemplatedMultiNode : public MultiNode
{

    TemplatedMultiNode(std::string name, std::string ros2_namespace): MultiNode(name, ros2_namespace)
    {
        _id = item_name_to_id(name);
        _name = name;
        _msg_size = 0;
    }


    ~TemplatedMultiNode()
    {
        this->_subscribers.clear();
        this->_publishers.clear();
        this->_clients.clear();
        this->_services.clear();
        this->_timers.clear();
    }


    void add_publisher(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
    {

        std::string topic_name = id_to_topic_name(id);

        add_publisher(topic_name, qos_profile);

    }


    void add_publisher(std::string topic_name, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
    {

        // to create the publisher I need a string and an integer id
        // the string is passed to the ROS2 create_publisher API, while the int is used for indexing
        int id = item_name_to_id(topic_name);

        // if I have already created a publisher with this id, do nothing
        auto it = _publishers.find(id);
        if (it == _publishers.end()){

            typename rclcpp::Publisher<MsgType>::SharedPtr publisher = this->create_publisher<MsgType>(
                topic_name,
                qos_profile);

            // store the publisher into the map structure
            _publishers[id] = publisher;

            rclcpp::Logger logger = this->get_logger();
            RCLCPP_INFO(logger, "Created publisher to %s", topic_name.c_str());
        }

        this->stats.start_time = this->now().nanoseconds();
    }


    void add_subscriber(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
    {

        std::string topic_name = id_to_topic_name(id);

        add_subscriber(topic_name, qos_profile);

    }


    void add_subscriber(std::string topic_name, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
    {

        // to create the subscriber I need a string and an integer id
        // the string is passed to the ROS2 create_subscription API, while the int is used for indexing
        int id = item_name_to_id(topic_name);

        // if I have already created a subscriber with this id, do nothing
        auto it = _subscribers.find(id);
        if (it == _subscribers.end()){

            std::function<void(const typename MsgType::SharedPtr msg)> fcn = std::bind(&TemplatedMultiNode::topic_callback, this, std::placeholders::_1, id);

            typename rclcpp::Subscription<MsgType>::SharedPtr subscriber = this->create_subscription<MsgType>(
                topic_name,
                fcn,
                qos_profile);

            // store the subscriber into the map structure
            _subscribers[id] = subscriber;

            this->stats.durations_map[id] = NodeStats::IterativeStats();

            rclcpp::Logger logger = this->get_logger();
            RCLCPP_INFO(logger, "Created subscription to %s", topic_name.c_str());

        }

        this->stats.start_time = this->now().nanoseconds();
    }


    void add_service(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
    {

        // to create the service I need a string and an integer id
        // the string is passed to the ROS2 create_service API, while the int is used for indexing
        std::string service_name = id_to_service_name(id);

        // if I have already created a service with this id, do nothing
        auto it = _services.find(id);
        if (it == _services.end()){

            rclcpp::Service<performance_test::srv::GetHeader>::SharedPtr service;
            service = this->create_service<performance_test::srv::GetHeader>(
                service_name,
                std::bind(&TemplatedMultiNode::service_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                qos_profile);

            // store the service into the map structure
            _services[id] = service;

            rclcpp::Logger logger = this->get_logger();
            RCLCPP_INFO(logger, "Created server to %s", service_name.c_str());
        }

        this->stats.start_time = this->now().nanoseconds();
    }


    void add_client(int id, rmw_qos_profile_t qos_profile = rmw_qos_profile_default)
    {

        // to create the client I need a string and an integer id
        // the string is passed to the ROS2 create_client API, while the int is used for indexing
        std::string service_name = id_to_service_name(id);

        // if I have already created a client with this id, do nothing
        auto it = _clients.find(id);
        if (it == _clients.end()){

            rclcpp::Client<performance_test::srv::GetHeader>::SharedPtr client;
            client = this->create_client<performance_test::srv::GetHeader>(
                service_name,
                qos_profile);

            // store the client into the map structure
            _clients[id] = client;

            this->stats.durations_map[id] = NodeStats::IterativeStats();

            rclcpp::Logger logger = this->get_logger();

            while (!client->wait_for_service(std::chrono::milliseconds(100))) {
                if (!rclcpp::ok()) {
                    rclcpp::Logger logger = this->get_logger();
                    RCLCPP_ERROR(logger, "Interrupted while waiting for the service.");
                    assert(0);
                }
                RCLCPP_INFO(logger, "Waiting for service %s to appear...", service_name.c_str());
            }

            RCLCPP_INFO(logger, "Created client to %s", service_name.c_str());
        }

        this->stats.start_time = this->now().nanoseconds();
    }


    void add_timer(float frequency, std::function< void() > callback)
    {

        // convert provided frequency (e.g. 100 hertz) into a milliseconds period (e.g. 10 ms)
        int period = (1000/frequency);
        std::chrono::milliseconds ms_period = std::chrono::milliseconds(period);

        // TODO this should be substituting what is done at the beginning of simple_publisher_task
        // should do it also for clients
        for (auto const& map_item : _publishers){
            int id = map_item.first;
            this->stats.send_frequency_map[id] = frequency;
        }

        rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(ms_period, callback);

        _timers.push_back(timer);

        this->stats.start_time = this->now().nanoseconds();
    }


    void simple_spin_task(float frequency)
    {
        // store the frequency at which the node is spinning
        for (auto const& map_item : _subscribers){
            int id = map_item.first;
            this->stats.spin_frequency_map[id] = frequency;
        }

        // set task start time
        this->stats.start_time = this->now().nanoseconds();

        // spin, whether using the blocking (rclcpp::spin) or non blocking (rclcpp::spin_some) function
        if (frequency == -1){
            rclcpp::spin(this->get_node_base_interface());
        }
        else {
            rclcpp::WallRate loop(frequency);
            while (rclcpp::ok()){
                rclcpp::spin_some(this->get_node_base_interface());
                loop.sleep();
            }
        }

    }


    void publish_messages(float frequency, int task_duration_sec, int size = 0)
    {

        // loop on all the publishers registered in this node
        // each publisher will send 1 message
        for (auto const& map_item : _publishers){

            // get the publisher and the id of the topic where it will publish
            int id = map_item.first;
            auto publisher = map_item.second;

            // create a new messabe object
            typename MsgType::SharedPtr message(new MsgType());

            // if this is a MsgType with dynamic field, then resize it
            resize_if_possible<MsgType>(message, size);

            // attach some information to the message, to be parsed by the subscribers
            std::stringstream ss;
            ss <<"communication_id:"<< _id <<"\n"; // NOTE: this is the id of the publishing node!
            ss <<"send_frequency:" << frequency <<"\n";
            message->header.frame_id = ss.str().c_str();

            // attach publish time stamp to the message header
            rclcpp::Time message_publish_time = this->now();
            message->header.stamp = message_publish_time;

            // set task start time
            if (this->stats.durations_map[id].count == 0){
                this->stats.start_time = message_publish_time.nanoseconds();
            }

            // check task duration, i.e. stop publishing when time is exceeded
            int64_t duration_seconds = RCL_NS_TO_S(static_cast<int64_t>(message_publish_time.nanoseconds() - this->stats.start_time));
            if (duration_seconds >= task_duration_sec){
                return;
            }

            // publish the created message
            publisher->publish(message);

            // update the duration map (this is a publisher, the map will only contain the message count)
            this->stats.durations_map[id].update(0);

            //rclcpp::Logger logger = this->get_logger();
            //RCLCPP_DEBUG(logger, "Topic %d: published msg %d", id, this->stats.durations_map[id].count);

            // compute message size in a way that works for both vector and array
            auto begin = std::begin(message->data);
            auto end = std::end(message->data);
            _msg_size = end - begin;

        }

    }



    /**
     * The resize_if_possible method has two different implementations, depending on the MsgType of the TemplatedMultiNode
     * If the MsgType is performance_test::msg::StampedVector then the message passed as argument contains a dynamic field and it can be resized.
     * If the MsgType is everything else, then the method will not do anything.
     */

    // Compile only if MsgType is StampedVector
    template <class MsgType_Q>
    typename std::enable_if<(std::is_same<MsgType_Q, performance_test::msg::StampedVector>::value)>::type
    resize_if_possible(typename MsgType_Q::SharedPtr message, int size)
    {
        message->data.resize(size);
    }


    // Compile only if MsgType is NOT StampedVector
    template <class MsgType_Q>
    typename std::enable_if<(!std::is_same<MsgType_Q, performance_test::msg::StampedVector>::value)>::type
    resize_if_possible(typename MsgType_Q::SharedPtr message, int size)
    {
        (void)message;
        (void)size;
        return;
    }


    void simple_publisher_task(float frequency, int task_duration_sec, int msg_size = 0)
    {

        // store the publishing frequency of the node
        for (auto const& map_item : _publishers){
            int id = map_item.first;
            this->stats.send_frequency_map[id] = frequency;
        }

        // call the publish_messages method until rclcpp::shutdown()
        rclcpp::WallRate loop_rate(frequency);
        while(rclcpp::ok()){

            publish_messages(frequency, task_duration_sec, msg_size);

            loop_rate.sleep();
        }

    }


    void send_requests(float frequency, int task_duration_sec)
    {

        (void)frequency;

        // loop on all the clients registered in this node
        // each client will send 1 request
        for (auto const& map_item : _clients){

            // get the client and the id of the service to which it will request
            int id = map_item.first;
            auto client = map_item.second;

            // create a new request object
            auto request = std::make_shared<performance_test::srv::GetHeader::Request>();

            // attach request time stamp to the request header
            rclcpp::Time request_send_time = this->now();
            request->header.stamp = request_send_time;

            // set task start time
            if (this->stats.durations_map[id].count == 0){
                this->stats.start_time = request_send_time.nanoseconds();
            }

            // check task duration
            int64_t duration_seconds = RCL_NS_TO_S(static_cast<int64_t>(request_send_time.nanoseconds() - this->stats.start_time));
            if (duration_seconds >= task_duration_sec){
                return;
            }

            // send the request and wait for the response
            rclcpp::Client<performance_test::srv::GetHeader>::SharedFuture result_future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
                rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                continue;
            }

            // get the time at which the response is received
            rclcpp::Time response_receive_time = this->now();

            // compute the latency
            // i.e. the delta between when the response has been received and  when the request has been sent
            rclcpp::Duration rcl_duration = response_receive_time - request_send_time;
            int64_t request_duration_microseconds = RCL_NS_TO_US(static_cast<int64_t>(rcl_duration.nanoseconds()));

            // update the durations_map for this client with the last latency
            this->stats.durations_map[id].update(request_duration_microseconds);
        }

    }


    void simple_client_task(float frequency, int task_duration_sec)
    {

        // store the request frequency of the node
        for (auto const& map_item : _clients){
            int id = map_item.first;
            this->stats.send_frequency_map[id] = frequency;
        }

        // call the send_requests method until rclcpp::shutdown()
        rclcpp::WallRate loop_rate(frequency);
        while(rclcpp::ok()){

            send_requests(frequency, task_duration_sec);

            loop_rate.sleep();
        }
    }


    // default service handler
    void service_handler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<performance_test::srv::GetHeader::Request> request,
        const std::shared_ptr<performance_test::srv::GetHeader::Response> response)
    {

        (void)request_header;
        (void)request;

        response->header.stamp = this->now();

    }


    // default topic callback
    void topic_callback(const typename MsgType::SharedPtr msg, int id)
    {

        // get the time at which the callback is triggered
        rclcpp::Time received_time = this->now();

        // compute the latency
        // i.e. the delta between when the message has been received and the timestamp denoting its publish time
        rclcpp::Duration rcl_duration = received_time - rclcpp::Time(msg->header.stamp);
        int64_t delta_microseconds = RCL_NS_TO_US(static_cast<int64_t>(rcl_duration.nanoseconds()));

        // update the durations_map for this subscriber with the last latency
        this->stats.durations_map[id].update(delta_microseconds);

        //rclcpp::Logger logger = this->get_logger();
        //RCLCPP_DEBUG(logger, "Topic %d: received msg %d", id, this->stats.durations_map[id].count);

        // when I receive the first message on topic `id` store the delta time from node startup
        if (this->stats.durations_map[id].count == 1){

            uint64_t received_time_nanoseconds = received_time.nanoseconds();
            int64_t delta_microseconds = RCL_NS_TO_US(static_cast<int64_t>(received_time_nanoseconds - this->stats.start_time));
            this->stats.first_message_delta[id] = delta_microseconds;
        }

        // parse the string contained in the message to extract additional info
        std::map<std::string, std::string> m;
        std::string key, val;
        std::istringstream iss(msg->header.frame_id);
        while(std::getline(std::getline(iss, key, ':') >> std::ws, val))
            m[key] = val;

        this->stats.send_frequency_map[id] = std::stof(m["send_frequency"]);

        // compute message size in a way that works for both vector and array
        auto begin = std::begin(msg->data);
        auto end = std::end(msg->data);
        _msg_size = end - begin;

    }


    // ros2 communication tools
    std::map<int, typename rclcpp::Subscription<MsgType>::SharedPtr> _subscribers;
    std::map<int, typename rclcpp::Publisher<MsgType>::SharedPtr> _publishers;
    std::map<int, rclcpp::Client<performance_test::srv::GetHeader>::SharedPtr> _clients;
    std::map<int, rclcpp::Service<performance_test::srv::GetHeader>::SharedPtr> _services;

    std::vector<rclcpp::TimerBase::SharedPtr> _timers;

};

#endif // __TEMPLATED_MULTI_NODE_HPP__
