/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_METRICS__DUMMY_WORK_HPP_
#define PERFORMANCE_METRICS__DUMMY_WORK_HPP_

#include <chrono>

namespace performance_metrics
{

void dummy_work(std::chrono::microseconds work_duration);

}  // namespace performance_metrics

#endif  // PERFORMANCE_METRICS__DUMMY_WORK_HPP_
