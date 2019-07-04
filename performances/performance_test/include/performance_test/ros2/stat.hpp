/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <cmath>
#include <utility>

namespace performance_test {

// Use shifted data variance algorithm
// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
template <typename T>
class Stat
{
public:
  void add_sample(T x)
  {

    if (_n == 0) {
      _max = x;
      _min = x;
      K = x;
    }

    if (x > _max) {
      _max = x;
    }

    if (x < _min) {
      _min = x;
    }

    _n++;
    Ex += x - K;
    Ex2 += (x - K) * (x - K);
  }

  double mean() const
  {
    if (_n == 0) {
      return std::nan("");
    }
    return K + Ex / _n;
  }

  double stddev() const
  {
    if (_n == 0) {
      return std::nan("");
    }
    if (_n == 1) {
      return 0;
    }
    return std::sqrt((Ex2 - (Ex * Ex) / _n) / (_n));
  }

  double max() const
  {
    if (_n == 0) {
      return std::nan("");
    }
    return _max;
  }

  double min() const
  {
    if (_n == 0) {
      return std::nan("");
    }
    return _min;
  }

  void increment_tracking_number(void)
  {
     _n++;
  }

  unsigned long int n() const { return _n; }

private:
  double _max;
  double _min;
  double  K;
  double Ex = 0;
  double Ex2 = 0;
  long unsigned int _n = 0;
};
}