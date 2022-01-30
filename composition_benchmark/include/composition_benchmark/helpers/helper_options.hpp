#pragma once

#include <stdlib.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

struct CompositionOptions
{
  CompositionOptions(int argc, char ** argv)
  {
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

    // All arguments are in pairs + first argument should be program name
    assert((args.size() % 2) != 0);

    for (size_t i = 1; i < args.size(); i += 2) {
      std::string key = args[i];
      std::string str_value = args[i + 1];

      if (key == "subs") {
        assert(num_subs == nullptr);
        num_subs = std::make_unique<size_t>(atoi(str_value.c_str()));
      } else if (key == "freq") {
        assert(pub_frequency == nullptr);
        pub_frequency = std::make_unique<int>(atoi(str_value.c_str()));
      } else if (key == "size") {
        assert(msg_size == nullptr);
        msg_size = std::make_unique<int>(atoi(str_value.c_str()));
      } else if (key == "ipc") {
        assert(use_ipc == nullptr);
        use_ipc = std::make_unique<bool>(static_cast<bool>(atoi(str_value.c_str())));
      } else if (key == "spin_t") {
        assert(spin_type == nullptr);
        spin_type = std::make_unique<std::string>(str_value);
      } else if (key == "name") {
        assert(name == nullptr);
        name = std::make_unique<std::string>(str_value);
      } else {
        std::cout<<"INVALID KEY  "<< key << " WITH VALUE " << str_value << std::endl;
        assert(0);
      }
    }
  }

  std::unique_ptr<size_t> num_subs;
  std::unique_ptr<int> pub_frequency;
  std::unique_ptr<int> msg_size;
  std::unique_ptr<bool> use_ipc;
  std::unique_ptr<std::string> spin_type;
  std::unique_ptr<std::string> name;
};
