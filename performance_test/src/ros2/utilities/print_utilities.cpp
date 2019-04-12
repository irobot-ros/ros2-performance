#include "performance_test/ros2/print_utilities.hpp"
#include <fstream>
#include "math.h"


void print_node_stats(std::string filename, std::vector<std::shared_ptr<MultiNode>> nodes, std::string msg_type, int experiment_duration)
{

    std::ofstream out_file;

    if (filename.empty()){
        std::cout<<"WARNING: provided an empty filename! Results will not be stored!!"<<std::endl;
    }
    else {
        //open output stream
        out_file.open(filename);

        // write header to csv
        out_file <<
        "node_id\tcommunication_id\tavg_latency\tstd_dev\tmsg_count\tspin_frequency\tsend_frequency\tmsg_type\tmsg_size\tduration\tfirst_delta\n";
    }


    // write data for each node to file
    // each line represents 1 communication interface (e.g. pub,sub,client,service) on a particular node
    for (std::shared_ptr<MultiNode> node : nodes){

        int node_id = node->get_id();
        std::string node_name = node->get_name();

        auto durations_map = node->stats.durations_map;

        for (auto const& map_item : durations_map){

            int communication_id = map_item.first;
            auto iterative_stats = map_item.second;

            auto mean = iterative_stats.mean;
            auto variance = iterative_stats.variance;
            auto std_dev = sqrt(variance);
            auto count = iterative_stats.count;

            auto msg_size = node->get_message_size();

            int spin_frequency = node->stats.spin_frequency_map[communication_id];
            int send_frequency = node->stats.send_frequency_map[communication_id];

            int64_t first_delta = node->stats.first_message_delta[communication_id];

            // print to screen some info
            std::cout<<
            node_name << ": communication_id "<< communication_id <<
            " --> Mean: "<< mean << " StdDev: "<< std_dev<<" Count: "<< count<< " Delta: "<< first_delta<< std::endl;

            // eventually print to file the tab separated values
            if (out_file.is_open()){
                out_file <<
                node_id<<"\t"<< communication_id <<"\t"<< mean<<"\t"<<std_dev<<"\t"<<
                count<<"\t"<<spin_frequency<<"\t"<<send_frequency<<"\t"<<msg_type<<"\t"<<msg_size<<"\t"<<
                experiment_duration<<"\t"<<first_delta<<
                "\n";
            }
        }

    }

    // eventually close the output stream
    if (out_file.is_open()){
        out_file.close();
    }

}
