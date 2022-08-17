/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__COMMUNICATION_HPP_
#define PERFORMANCE_TEST__COMMUNICATION_HPP_

#include <cassert>
#include <string>

namespace performance_test
{

enum class msg_pass_by_t
{
  PASS_BY_UNIQUE_PTR,
  PASS_BY_SHARED_PTR,
  PASS_BY_LOANED_MSG,
};

inline msg_pass_by_t string_to_msg_pass_by(const std::string & str)
{
  if (str == "unique_ptr") {
    return msg_pass_by_t::PASS_BY_UNIQUE_PTR;
  } else if (str == "shared_ptr") {
    return msg_pass_by_t::PASS_BY_SHARED_PTR;
  } else if (str == "loaned_msg") {
    return msg_pass_by_t::PASS_BY_LOANED_MSG;
  } else {
    assert(0);
    return msg_pass_by_t::PASS_BY_UNIQUE_PTR;
  }
}

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__COMMUNICATION_HPP_
