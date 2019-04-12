#include "performance_test/ros2/options.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>

bool find_command_option(const std::vector<std::string> &args, const std::string &option)
{
    return std::find(args.begin(), args.end(), option) != args.end();
}


std::string get_command_option(const std::vector<std::string> &args, const std::string &option)
{
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end() && ++it != args.end())
    {
        return *it;
    }
    return std::string();
}


bool get_flag_option(const std::vector<std::string> &args, const std::string &option)
{
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end())
    {
        return true;
    }
    return false;
}


bool parse_command_options(
    int argc, char **argv,
    int *n_publishers, int *n_subscribers,
    int *n_clients, int *n_services,
    std::string *msg_type,
    int *msg_size,
    int *executors,
    float *frequency,
    int *experiment_duration,
    std::string *filename,
    std::string *ros_namespace)
{
    std::vector<std::string> args(argv, argv + argc);

    // print help instructions
    if (find_command_option(args, "-h") || find_command_option(args, "-help") || find_command_option(args, "--h") || find_command_option(args, "--help"))
    {
        std::stringstream ss;
        ss << "Usage:" << std::endl;
        ss << " -h, --h, -help, --help: This message." << std::endl;
        ss << " --pubs: Publishers number." << std::endl;
        ss << " --subs: Subscribers number." << std::endl;
        ss << " --clients: Clients number." << std::endl;
        ss << " --services: Services number." << std::endl;
        ss << " --msg_type: Type of messages used for topics or services." << std::endl;
        ss << " --executors: Let the node spin in single or multithread executors." << std::endl;
        ss << " --frequency: Nodes spinning/publishing/requesting frequency." << std::endl;
        ss << " --filename: Provide a file path if you want to save statistics." << std::endl;
        ss << " --ros_namespace: Provide a ROS namespace." << std::endl;
        ss << " --duration: Tests duration." << std::endl;
        std::cout << ss.str();
        return false;
    }

    // number of command line arguments correctly parsed.
    // it has to be equal to the total number of arguments otherwise the function will return false
    int n_valid_arguments = 0;

    // check for the eventual presence of any recognized command line argument
    // if the user provides not required arguments, return false

    auto n_publishers_str = get_command_option(args, "--pubs");
    if (!n_publishers_str.empty())
    {
        if (n_publishers == nullptr)
        {
            std::cout << "ERROR: Provided command line option --pubs which is not used by this application. Exit\n";
            return false;
        }
        *n_publishers = std::stoi(n_publishers_str.c_str());
        n_valid_arguments++;
    }

    auto n_subscribers_str = get_command_option(args, "--subs");
    if (!n_subscribers_str.empty())
    {
        if (n_subscribers == nullptr)
        {
            std::cout << "ERROR: Provided command line option --subs which is not used by this application. Exit\n";
            return false;
        }
        *n_subscribers = std::stoi(n_subscribers_str.c_str());
        n_valid_arguments++;
    }

    auto n_clients_str = get_command_option(args, "--clients");
    if (!n_clients_str.empty())
    {
        if (n_clients == nullptr)
        {
            std::cout << "ERROR: Provided command line option --clients which is not used by this application. Exit\n";
            return false;
        }
        *n_clients = std::stoi(n_clients_str.c_str());
        n_valid_arguments++;
    }

    auto n_services_str = get_command_option(args, "--services");
    if (!n_services_str.empty())
    {
        if (n_services == nullptr)
        {
            std::cout << "ERROR: Provided command line option --services which is not used by this application. Exit\n";
            return false;
        }
        *n_services = std::stoi(n_services_str.c_str());
        n_valid_arguments++;
    }

    auto msg_type_str = get_command_option(args, "--msg_type");
    if (!msg_type_str.empty())
    {
        if (msg_type == nullptr)
        {
            std::cout << "ERROR: Provided command line option --msg_type which is not used by this application. Exit\n";
            return false;
        }
        *msg_type = msg_type_str;
        n_valid_arguments++;
    }

    auto msg_size_str = get_command_option(args, "--msg_size");
    if (!msg_size_str.empty())
    {
        if (msg_size == nullptr)
        {
            std::cout << "ERROR: Provided command line option --msg_size which is not used by this application. Exit\n";
            return false;
        }
        *msg_size = std::stoi(msg_size_str.c_str());
        n_valid_arguments++;
    }

    auto executors_str = get_command_option(args, "--executors");
    if (!executors_str.empty())
    {
        if (executors == nullptr)
        {
            std::cout << "ERROR: Provided command line option --executors which is not used by this application. Exit\n";
            return false;
        }
        *executors = std::stoi(executors_str.c_str());
        n_valid_arguments++;
    }

    auto frequency_str = get_command_option(args, "--frequency");
    if (!frequency_str.empty())
    {
        if (frequency == nullptr)
        {
            std::cout << "ERROR: Provided command line option --frequency which is not used by this application. Exit\n";
            return false;
        }
        *frequency = std::stof(frequency_str.c_str());
        n_valid_arguments++;
    }

    auto filename_str = get_command_option(args, "--filename");
    if (!filename_str.empty())
    {
        if (filename == nullptr)
        {
            std::cout << "ERROR: Provided command line option --filename which is not used by this application. Exit\n";
            return false;
        }
        *filename = filename_str;
        n_valid_arguments++;
    }

    auto ros_namespace_str = get_command_option(args, "--ros_namespace");
    if (!ros_namespace_str.empty())
    {
        if (ros_namespace == nullptr)
        {
            std::cout << "ERROR: Provided command line option --ros_namespace which is not used by this application. Exit\n";
            return false;
        }
        *ros_namespace = ros_namespace_str;
        n_valid_arguments++;
    }

    auto experiment_duration_str = get_command_option(args, "--duration");
    if (!experiment_duration_str.empty())
    {
        if (experiment_duration == nullptr)
        {
            std::cout << "ERROR: Provided command line option --duration which is not used by this application. Exit\n";
            return false;
        }
        *experiment_duration = std::stoi(experiment_duration_str.c_str());
        n_valid_arguments++;
    }

    if (n_valid_arguments < (argc-1)/2.0 ){
        std::cout << "ERROR: provided "<< (argc-1)/2.0 << " command line arguments, but only "<< n_valid_arguments << " have been parsed correctly. Exit\n";
        return false;
    }

    return true;
}