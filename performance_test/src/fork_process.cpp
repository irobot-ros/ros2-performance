/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <sys/types.h>
#include <unistd.h>

#include <performance_test/fork_process.hpp>

size_t fork_process(size_t num_processes)
{
  pid_t pid = getpid();
  size_t i = 0;
  for (i = 0; i < num_processes; i++) {
    // Fork only the for the first (n-1) processes
    if (i != num_processes - 1) {
      pid = fork();
      // If is a child process, break
      if (pid == 0) {
        break;
      }
    } else {
      break;
    }
  }

  return i;
}