# How to create a new topology for the iRobot Performance Framework

A "topology" is a syntetic description of a ROS2 process, written in a format that can be parsed by the `performance_test_factory` package.

Here you can find an example of a [simple topology](examples/simple_architecture.json).

We chose to use json files, since they are easy to write and also nice to visualize.

## How to write a new json topology

This is the structure of the json:
 - The root contains a `nodes` field. This is a list of elements where each of them describes a specific type of node.
 - A node element has the following fields:
    - The unique name of the node must be specified as `node_name`. This field is mandatory.
    - Then you have the following optional fields: `publishers`, `subscribers`, `clients`, `servers`. Each of them contains a list of elements.
    - The last optional field is named `number`. This field takes an integer value and this represents how many copies of this node you want to create in the system. Assuming that you specified the name as `"node_name": "my_node_name"`, then the different copies of the node will be named as `my_node_name_1`, `my_node_name_2`, etc.
 - Each of the elements in the `publishers`, `subscribers`, `clients` or `servers` lists expect more or less the same fields.
     - The `topic_name` or `service_name`. This field is mandatory.
     - The `msg_type` or `srv_type`. This is a string that defines the type of message used. More details on how to specify the name of a specific message can be found later. This field is mandatory.
     - If the interface specified in the `msg_type` or `srv_type` has a vector field, the size of this vector should be specified using the `msg_size` field. The size is in bytes.
     - Publishers and clients have to specify the `period_ms` field. This denotes the amount of milliseconds between consecutive messages are published or service requests are issued. This field is mandatory for publishers and clients.
     - For all the entity types, quality of service options can be specified in the json format.


## How to use custom messages in a topology

The `msg_type` field of the json file will only accept names of interfaces that are known to the factory.
Such interfaces must be defined as part of the `performance_test_factory_plugins`.

There are 2 constraints on the custom interfaces that can be used in the performance framework:
 - Each interface must contain a field named
 ```
 performance_test_msgs/PerformanceHeader header
 ```
 - Each interface must contain a field named
  ```
  byte[] data
  ```
  The `data` field can be both a vector (as in the example) or an array. This simulates the payload of the message.

An example of such a plugin package is the [irobot_interfaces_plugin](../irobot_interfaces_plugin).
At a first glance, this package may look identical to any of the ROS2 packages that define custom interfaces, but there are 2 small differences:

 - In the `package.xml` file it's necessary to add the line
 ```
  <member_of_group>performance_test_factory_plugins</member_of_group>
 ```
   This is used to let the compiler know that the `performance_test_factory` will have to support these interfaces.
 - In the `CMakeLists.txt` file it's necessary to add a dependency on `performance_test`
 ```
  find_package(performance_test REQUIRED)
 ```
   And to call a custom CMake function to generate a support library for the factory.
 ```
  generate_factory_plugin("${MSGS}" "${SRVS}")
 ```
   Where the arguments are CMake variables containing a list of messages and services names. Note that the `"` are needed to pass each list as a single argument.

#### How to reference a custom message in a topology

Once you have created and built your plugins package, you can set the `msg_type` value to the name of a message created in that package.
The value of the `msg_type` field has to be a string containing the name of the plugin and a lower-case version of the name of the message.

Let's assume that you create a plugin package named `my_factory_plugin`. This package defines the following interfaces: `RobotPose.msg`, `Kinematic10dof.msg` and `Odometry2Wheels.srv`.
The respective types would be: `my_factory_plugin::robot_pose`, `my_factory_plugin::kinematic10dof`, `my_factory_plugin::odometry2_wheels`.
The interface name has to be lower-cased with the same logic used when including it the message in a C++ application.