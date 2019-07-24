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


def get_interface_name_from_path(interface_path):
  '''
  path has the form msg/PoseWithCovariance.msg
  the name is only PoseWithCovariance
  '''
  _, name = interface_path.split('/', 1)
  name = name[:-4]
  return name


def get_lowercased_name(interface_name):
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


def add_include_paths(msgs, srvs, package):

  content = """

  """

  for msg_name in msgs:
    statement = get_cpp_include_statement(msg_name, package, "msg")
    content += statement + "\n"
  content += "\n"
  for srv_name in srvs:
    statement = get_cpp_include_statement(srv_name, package, "srv")
    content += statement + "\n"

  return content


def add_sub_factory(msgs, package):

  if len(msgs) == 0:
    return ""

  content = """

  extern "C" void add_subscriber_impl(
    std::shared_ptr<performance_test::Node> n,
    std::string msg_type,
    std::string topic_name,
    performance_test::Tracker::TrackingOptions tracking_options,
    msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t custom_qos_profile)
  {
    const std::map<std::string, std::function<void()>> subscribers_factory{
  """

  for msg_name in msgs:

    msg_class_name = get_namespaced_cpp_class_name(msg_name, package, "msg")
    topic = "performance_test::Topic<" + msg_class_name + ">"
    lowercased_name = get_lowercased_name(msg_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = "{" + map_key +",  [&] { n->add_subscriber(" + topic +"(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };

    if (subscribers_factory.find(msg_type) == subscribers_factory.end()){
      assert(0 && "unknown msg type passed to factory method!" );
    }

    subscribers_factory.at(msg_type)();
  }

  """

  return content


def add_pub_factory(msgs, package):

  if len(msgs) == 0:
    return ""

  content = """

  extern "C" void add_publisher_impl(
    std::shared_ptr<performance_test::Node> n,
    std::string msg_type,
    std::string topic_name,
    msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t custom_qos_profile,
    std::chrono::milliseconds period_ms,
    size_t msg_size)
  {
    const std::map<std::string, std::function<void()>> publishers_factory{
  """

  for msg_name in msgs:

    msg_class_name = get_namespaced_cpp_class_name(msg_name, package, "msg")
    topic = "performance_test::Topic<" + msg_class_name + ">"
    lowercased_name = get_lowercased_name(msg_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = "{" + map_key +",  [&] { n->add_periodic_publisher(" + topic +"(topic_name), period_ms, msg_pass_by, custom_qos_profile); } },"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };

    if (publishers_factory.find(msg_type) == publishers_factory.end()){
      assert(0 && "unknown msg type passed to factory method!" );
    }

    publishers_factory.at(msg_type)();
  }

  """

  return content


def add_server_factory(srvs, package):

  if len(srvs) == 0:
    return ""

  content = """

  extern "C" void add_server_impl(
    std::shared_ptr<performance_test::Node> n,
    std::string srv_type,
    std::string service_name,
    rmw_qos_profile_t custom_qos_profile)
  {
    const std::map<std::string, std::function<void()>> servers_factory{
  """

  for srv_name in srvs:

    srv_class_name = get_namespaced_cpp_class_name(srv_name, package, "srv")
    service = "performance_test::Service<" + srv_class_name + ">"
    lowercased_name = get_lowercased_name(srv_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = "{" + map_key +",  [&] { n->add_server(" + service +"(service_name), custom_qos_profile); } },"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };

    if (servers_factory.find(srv_type) == servers_factory.end()){
      assert(0 && "unknown srv type passed to factory method!" );
    }

    servers_factory.at(srv_type)();
  }

  """

  return content


def add_client_factory(srvs, package):

  if len(srvs) == 0:
    return ""

  content = """

  extern "C" void add_client_impl(
    std::shared_ptr<performance_test::Node> n,
    std::string srv_type,
    std::string service_name,
    rmw_qos_profile_t custom_qos_profile,
    std::chrono::milliseconds period_ms)
  {
    const std::map<std::string, std::function<void()>> clients_factory{
  """

  for srv_name in srvs:

    srv_class_name = get_namespaced_cpp_class_name(srv_name, package, "srv")
    service = "performance_test::Service<" + srv_class_name + ">"
    lowercased_name = get_lowercased_name(srv_name)
    map_key = "\"" + lowercased_name + "\""

    map_entry = "{" + map_key +",  [&] { n->add_periodic_client(" + service +"(service_name), period_ms, custom_qos_profile); } },"

    content += "\n" + map_entry

  if content.endswith(","):
    content = content[:-1]

  content += """
    };

    if (clients_factory.find(srv_type) == clients_factory.end()){
        assert(0 && "unknown srv type passed to factory method!" );
    }

    clients_factory.at(srv_type)();
}

  """

  return content


def main():

  parser = argparse.ArgumentParser(description='Python script for generating interfaces implementation')
  parser.add_argument('output_path', type=str)
  parser.add_argument('--package', type=str)
  parser.add_argument('--msg', type=str, nargs='+', default=[])
  parser.add_argument('--srv', type=str, nargs='+', default=[])

  args = parser.parse_args()

  output_file_path = args.output_path
  package = args.package
  msg_paths = args.msg
  srv_paths = args.srv

  if not msg_paths and not srv_paths:
    sys.exit('No interfaces!')

  msgs = []
  srvs = []

  for path in msg_paths:
    msgs.append(get_interface_name_from_path(path))
  for path in srv_paths:
    srvs.append(get_interface_name_from_path(path))

  outdir = os.path.dirname(output_file_path)
  os.makedirs(outdir, exist_ok=True)

  content = """
  #include "performance_test/ros2/node.hpp"

  """

  content += add_include_paths(msgs, srvs, package)
  content += add_sub_factory(msgs, package)
  content += add_pub_factory(msgs, package)
  content += add_server_factory(srvs, package)
  content += add_client_factory(srvs, package)

  #change = False

  def create(filename, content):
    if os.path.exists(filename):
      old_content = open(filename, 'r').read()
      #if old_content == content:
        #return
    #global change
    #change = True
    open(filename, 'w').write(content)

  create(output_file_path, content)

  # Check changes
  #if not change:
    #sys.exit('No changes!')




if __name__ == "__main__":
  main()
