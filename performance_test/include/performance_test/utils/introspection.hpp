/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__UTILS__INTROSPECTION_HPP_
#define PERFORMANCE_TEST__UTILS__INTROSPECTION_HPP_

namespace performance_test
{

// SFINAE test to verify is a ROS 2 message has a `data` field
template<typename T>
class msg_has_data_field
{
  using one = char;
  struct two { char x[2]; };

  template<typename C> static one test(decltype(&C::set__data));
  template<typename C> static two test(...);

public:
  enum { value = sizeof(test<T>(0)) == sizeof(char) };
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILS__INTROSPECTION_HPP_
