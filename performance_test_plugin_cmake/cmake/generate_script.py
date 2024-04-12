#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2019, iRobot ROS
#  All rights reserved.
#
#  This file is part of ros2-performance, which is released under BSD-3-Clause.
#  You may use, distribute and modify this code under the BSD-3-Clause license.
#

import argparse
import os
import re
import sys
import textwrap

def get_interface_name_from_path(interface_path):
  '''
  path has the form msg/PoseWithCovariance.msg
  the name is only PoseWithCovariance
  '''

  # get filename from path
  name_with_extension = os.path.basename(interface_path)
  # remove extension from filename
  name = os.path.splitext(name_with_extension)[0]
  return name


def get_lowercased_name(interface_name):
  '''
  interface_name has the form PoseWithCovariance
  the lowercased version is pose_with_covariance
  '''

  # Remove consecutive upper-case letters: HelloWORLD becomes HelloWorld
  last_upper = False
  for idx, c in enumerate(interface_name):
    if c.isupper():
      if last_upper:
        interface_name = interface_name[:idx] + c.lower() + interface_name[idx+1:]
      last_upper = True
    else:
      last_upper = False

  # This regular expression splits the name at each capital letter
  parts = re.sub( r"([A-Z])", r" \1", interface_name).split()
  lowercase_parts = [x.lower() for x in parts]
  lowercased_name = "_".join(lowercase_parts)
  return lowercased_name


def get_cpp_include_statement(interface_name, package, interface_type):
  '''
  interface_name has the form PoseWithCovariance, in package geometry_msgs and its a "msg"
  the include statement will be
  #include "geometry_msgs/msg/pose_with_covariance.hpp"
  '''

  include_file_name = get_lowercased_name(interface_name) + ".hpp"

  statement = "#include \"" + package + "/" + interface_type + "/" + include_file_name + "\""
  return statement


def get_namespaced_cpp_class_name(interface_name, package, interface_type):
  '''
  interface_name has the form PoseWithCovariance, in package geometry_msgs and its a "msg"
  the namespaced cpp class name will be
  geometry_msgs::msg::PoseWithCovariance"
  '''

  class_name = package + "::" + interface_type + "::" + interface_name
  return class_name


def get_include_paths(msgs, srvs, actions, package):
  '''
  Add all necessary include directives
  '''

  content = """
  #include <map>
  #include <memory>
  #include <string>
  #include <rclcpp/rclcpp.hpp>
  #include "performance_metrics/tracker.hpp"
  #include "performance_test/performance_node_base.hpp"

  """

  for msg_name in msgs:
    statement = get_cpp_include_statement(msg_name, package, "msg")
    content += statement + "\n"
  content += "\n"
  for srv_name in srvs:
    statement = get_cpp_include_statement(srv_name, package, "srv")
    content += statement + "\n"
  content += "\n"
  for action_name in actions:
    statement = get_cpp_include_statement(action_name, package, "action")
    content += statement + "\n"

  return content

def get_sub_factory(msgs, package):

  if len(msgs) == 0:
    return ""

  content = """

  extern "C" void add_subscriber_impl(
  """

  performance_node = "performance_test::PerformanceNodeBase"
  content += "\n std::shared_ptr<" + performance_node + "> n,"

  content += """
    const std::string & msg_type,
    const std::string & topic_name,
    const performance_metrics::Tracker::Options & tracking_options,
    performance_test::msg_pass_by_t msg_pass_by,
    const rclcpp::QoS & custom_qos_profile)
  {
    const std::map<std::string, std::function<void()>> subscribers_factory{
  """

  function = "add_subscriber"
  user_args = "topic_name, msg_pass_by, tracking_options, custom_qos_profile"

  for msg_name in msgs:

    msg_class_name = get_namespaced_cpp_class_name(msg_name, package, "msg")
    lowercased_name = get_lowercased_name(msg_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = f"{{ {map_key}, [&] {{ n->{function}<{msg_class_name}>({user_args});}} }},"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };"""

  content += """

    if (subscribers_factory.find(msg_type) == subscribers_factory.end()) {
      throw std::runtime_error("unknown msg type passed to subscribers factory: " + msg_type);
    }

    subscribers_factory.at(msg_type)();
  }

  """

  return content

def get_pub_factory(msgs, package):

  if len(msgs) == 0:
    return ""

  content = """

  extern "C" void add_publisher_impl(
  """

  performance_node = "performance_test::PerformanceNodeBase"
  content += "\n std::shared_ptr<" + performance_node + "> n,"

  content += """
    const std::string & msg_type,
    const std::string & topic_name,
    performance_test::msg_pass_by_t msg_pass_by,
    const rclcpp::QoS & custom_qos_profile,
    std::chrono::microseconds period,
    size_t msg_size)
  {
    const std::map<std::string, std::function<void()>> publishers_factory{
  """

  function = "add_periodic_publisher"
  user_args = "topic_name, period, msg_pass_by, custom_qos_profile, msg_size"

  for msg_name in msgs:
    msg_class_name = get_namespaced_cpp_class_name(msg_name, package, "msg")

    lowercased_name = get_lowercased_name(msg_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = f"{{ {map_key}, [&] {{ n->{function}<{msg_class_name}>({user_args});}} }},"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };"""

  content += """

    if (publishers_factory.find(msg_type) == publishers_factory.end()) {
      throw std::runtime_error("unknown msg type passed to publishers factory: " + msg_type);
    }

    publishers_factory.at(msg_type)();
  }

  """

  return content

def get_server_factory(srvs, package):

  if len(srvs) == 0:
    return ""

  content = """

  extern "C" void add_server_impl(
  """

  performance_node = "performance_test::PerformanceNodeBase"
  content += "\n std::shared_ptr<" + performance_node + "> n,"

  content += """
    const std::string & srv_type,
    const std::string & service_name,
    const rclcpp::QoS & custom_qos_profile)
  {
    const std::map<std::string, std::function<void()>> servers_factory{
  """

  function = "add_server"
  user_args = "service_name, custom_qos_profile"

  for srv_name in srvs:

    srv_class_name = get_namespaced_cpp_class_name(srv_name, package, "srv")
    lowercased_name = get_lowercased_name(srv_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = f"{{ {map_key}, [&] {{ n->{function}<{srv_class_name}>({user_args});}} }},"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };"""

  content += """

    if (servers_factory.find(srv_type) == servers_factory.end()) {
      throw std::runtime_error("unknown srv type passed to servers factory: " + srv_type);
    }

    servers_factory.at(srv_type)();
  }

  """

  return content

def get_client_factory(srvs, package):

  if len(srvs) == 0:
    return ""

  content = """

  extern "C" void add_client_impl(
  """

  performance_node = "performance_test::PerformanceNodeBase"
  content += "\n std::shared_ptr<" + performance_node + "> n,"

  content += """
    const std::string & srv_type,
    const std::string & service_name,
    const rclcpp::QoS & custom_qos_profile,
    std::chrono::microseconds period)
  {
    const std::map<std::string, std::function<void()>> clients_factory{
  """

  function = "add_periodic_client"
  user_args = "service_name, period, custom_qos_profile"

  for srv_name in srvs:

    srv_class_name = get_namespaced_cpp_class_name(srv_name, package, "srv")
    lowercased_name = get_lowercased_name(srv_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = f"{{ {map_key}, [&] {{ n->{function}<{srv_class_name}>({user_args});}} }},"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };"""

  content += """
    if (clients_factory.find(srv_type) == clients_factory.end()) {
      throw std::runtime_error("unknown srv type passed to clients factory: " + srv_type);
    }

    clients_factory.at(srv_type)();
 }

  """

  return content

def get_action_server_factory(actions, package):

  if len(actions) == 0:
    return ""

  content = """

  extern "C" void add_action_server_impl(
  """

  performance_node = "performance_test::PerformanceNodeBase"
  content += "\n std::shared_ptr<" + performance_node + "> n,"

  content += """
    const std::string & action_type,
    const std::string & action_name,
    const rclcpp::QoS & custom_qos_profile)
  {
    const std::map<std::string, std::function<void()>> action_servers_factory{
  """

  function = "add_action_server"
  user_args = "action_name, custom_qos_profile"

  for action_name in actions:

    action_class_name = get_namespaced_cpp_class_name(action_name, package, "action")
    lowercased_name = get_lowercased_name(action_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = f"{{ {map_key}, [&] {{ n->{function}<{action_class_name}>({user_args});}} }},"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };"""

  content += """
    if (action_servers_factory.find(action_type) == action_servers_factory.end()) {
      throw std::runtime_error("unknown action type passed to action servers factory: " + action_type);
    }

    action_servers_factory.at(action_type)();
  }

  """

  return content

def get_action_client_factory(actions, package):

  if len(actions) == 0:
    return ""

  content = """

  extern "C" void add_action_client_impl(
  """

  performance_node = "performance_test::PerformanceNodeBase"
  content += "\n std::shared_ptr<" + performance_node + "> n,"

  content += """
    const std::string & action_type,
    const std::string & action_name,
    const rclcpp::QoS & custom_qos_profile,
    std::chrono::microseconds period)
  {
    const std::map<std::string, std::function<void()>> action_clients_factory{
  """

  function = "add_periodic_action_client"
  user_args = "action_name, period, custom_qos_profile"

  for action_name in actions:

    action_class_name = get_namespaced_cpp_class_name(action_name, package, "action")
    lowercased_name = get_lowercased_name(action_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = f"{{ {map_key}, [&] {{ n->{function}<{action_class_name}>({user_args});}} }},"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };"""

  content += """
    if (action_clients_factory.find(action_type) == action_clients_factory.end()) {
      throw std::runtime_error("unknown action type passed to action clients factory: " + action_type);
    }

    action_clients_factory.at(action_type)();
  }

  """

  return content

def main():

  parser = argparse.ArgumentParser(description='Python script for generating interfaces implementation')
  parser.add_argument('output_path', type=str)
  parser.add_argument('--package', type=str)
  parser.add_argument('--msg', type=str, nargs='+', default=[])
  parser.add_argument('--srv', type=str, nargs='+', default=[])
  parser.add_argument('--action', type=str, nargs='+', default=[])

  args = parser.parse_args()

  output_file_path = args.output_path
  package = args.package
  msg_paths = args.msg
  srv_paths = args.srv
  action_paths = args.action

  if not msg_paths and not srv_paths and not action_paths:
    sys.exit('No interfaces!')

  msgs = []
  srvs = []
  actions = []

  for path in msg_paths:
    msgs.append(get_interface_name_from_path(path))
  for path in srv_paths:
    srvs.append(get_interface_name_from_path(path))
  for path in action_paths:
    actions.append(get_interface_name_from_path(path))

  outdir = os.path.dirname(output_file_path)
  os.makedirs(outdir, exist_ok=True)

  content = ""
  content += get_include_paths(msgs, srvs, actions, package)
  content += get_sub_factory(msgs, package)
  content += get_pub_factory(msgs, package)
  content += get_server_factory(srvs, package)
  content += get_client_factory(srvs, package)
  content += get_action_server_factory(actions, package)
  content += get_action_client_factory(actions, package)

  # Remove wacky indentation
  content = textwrap.dedent(content)

  def create(filename, content):
    if os.path.exists(filename):
      old_content = open(filename, 'r').read()
      if old_content == content:
        return

    open(filename, 'w').write(content)

  create(output_file_path, content)

if __name__ == "__main__":
  main()
