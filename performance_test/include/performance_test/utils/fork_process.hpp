/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <cstddef>

namespace performance_test
{

/**
 * @brief Creates N-1 forks of the current process.
 * After running this function there will be N total processes (N-1 forked + 1 that invoked this function)
 * @param num_processes N total number of processes.
 * @return index of this process between 0 and N-1
 */ 
size_t fork_process(size_t num_processes);

}
