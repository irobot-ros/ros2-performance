#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/options.hpp"
#include "performance_test/ros2/print_utilities.hpp"
#include "performance_test/ros2/names_utilities.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // experiment default values
    int n_subscribers = 30;
    int n_publishers = 1;
    std::string msg_type = "10b";
    std::string ros_namespace = "";
    int executors = 0;
    bool verbose = false;


    bool ret = parse_command_options(
        argc, argv,
        &n_publishers,          // n_publishers
        &n_subscribers,         // n_subscribers
        nullptr,                // n_clients
        nullptr,                // n_services
        nullptr,                // msg_type
        nullptr,                // msg_size
        &executors,             // executors
        nullptr,                // frequency
        nullptr,                // experiment_duration
        nullptr,                // filename
        &ros_namespace          // namespace
    );

    if (!ret){
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executor::Executor::SharedPtr executor = nullptr;
    if (executors == 1){
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }

    std::cout<<"Start test"<<std::endl;

    TemplateFactory ros2system(ros_namespace);

    std::vector<std::shared_ptr<MultiNode>> sub_nodes = ros2system.create_subscribers(0, n_subscribers, n_publishers, msg_type, verbose);

    std::cout<<"Subscribers created!"<<std::endl;

    std::vector<std::shared_ptr<MultiNode>> pub_nodes = ros2system.create_publishers(n_subscribers, n_subscribers + n_publishers, msg_type, verbose);

    std::cout<<"Publishers created!"<<std::endl;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();


    if (executor != nullptr){
        std::thread thread([executor](){
            executor->spin();
        });
        thread.detach();
    }


    bool done = false;
    int best = 0;
    auto query_inteval = std::chrono::milliseconds(10);
    while (!done){
        done = true;
        std::this_thread::sleep_for(query_inteval);
        for (auto node : pub_nodes){

            int node_id = node->get_id();
            std::string topic_name = id_to_topic_name(node_id);

            int discovered = node->count_subscribers(topic_name);

            if (discovered > best){
                best = discovered;
                std::chrono::high_resolution_clock::time_point t_best = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t_best - t1 ).count();
                std::cout<<"Discovered --> "<<best <<"/"<<n_subscribers<<" after "<< duration << " milliseconds"<<std::endl;
            }

            if (discovered != n_subscribers){
                done = false;
                break;
            }
        }
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout<<"DISCOVERY TIME ----> "<< duration<<" milliseconds" <<std::endl;


    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;

    // store pointer to nodes into a unique vector
    std::vector<std::shared_ptr<MultiNode>> nodes_vector;
    nodes_vector.insert(nodes_vector.end(), sub_nodes.begin(), sub_nodes.end());
    nodes_vector.insert(nodes_vector.end(), pub_nodes.begin(), pub_nodes.end());
    print_node_stats("", nodes_vector, msg_type, 0);


    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout<<"End test"<<std::endl;

    return 0;
}


