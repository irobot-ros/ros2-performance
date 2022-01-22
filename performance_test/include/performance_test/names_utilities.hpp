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

namespace performance_test {

// converts a numeric id (e.g. 7) into a node name (e.g. node_7)
std::string id_to_node_name(int id);

// converts a numeric id (e.g. 7) into a service name (e.g. service_7)
std::string id_to_service_name(int id);

// converts a numeric id (e.g. 7) into a topic name (e.g. topic_7)
std::string id_to_topic_name(int id);

// extracts a numeric id from an item name (e.g. node_5, topic_10), eventually using hashing
int item_name_to_id(std::string name);

}