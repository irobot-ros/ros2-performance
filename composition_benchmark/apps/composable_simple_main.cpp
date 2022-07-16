/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include "composition_benchmark/composable_node.hpp"
#include "composition_benchmark/helpers/helper_factory.hpp"
#include "composition_benchmark/helpers/run_test.hpp"
#include "performance_test/executors.hpp"

int main(int argc, char ** argv)
{
  run_test(
    argc,
    argv,
    create_simple_nodes<ComposableNode>,
    [](const NodesVector&) {performance_test::sleep_task(MAX_HOURS);});
}
