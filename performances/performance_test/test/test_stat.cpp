/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include "performance_test/ros2/stat.hpp"



TEST(StatTest, StatInitTest)
{
  performance_test::Stat<unsigned long int> stat;

  EXPECT_TRUE(std::isnan(stat.mean()));
  EXPECT_TRUE(std::isnan(stat.stddev()));
  EXPECT_TRUE(std::isnan(stat.min()));
  EXPECT_TRUE(std::isnan(stat.max()));
  ASSERT_EQ((unsigned long int)0, stat.n());
}


TEST(StatTest, StatAddSamplesTest)
{
  performance_test::Stat<unsigned long int> stat;

  // Some samples
  unsigned long s1 = 100000;
  unsigned long s2 = 1000;
  unsigned long s3 = 10000;
  unsigned long s4 = 0;

  stat.add_sample(s1);

  ASSERT_DOUBLE_EQ((double)s1, stat.mean());
  ASSERT_DOUBLE_EQ((double)0, stat.stddev());
  ASSERT_DOUBLE_EQ((double)s1, stat.min());
  ASSERT_DOUBLE_EQ((double)s1, stat.max());

  stat.add_sample(s2);

  ASSERT_DOUBLE_EQ((double)50500, stat.mean());
  ASSERT_DOUBLE_EQ((double)49500, stat.stddev());
  ASSERT_DOUBLE_EQ((double)s2, stat.min());
  ASSERT_DOUBLE_EQ((double)s1, stat.max());

  stat.add_sample(s3);

  ASSERT_DOUBLE_EQ((double)37000, stat.mean());
  EXPECT_NEAR((double)44698.993277254, stat.stddev(), 1e-5);
  ASSERT_DOUBLE_EQ((double)s2, stat.min());
  ASSERT_DOUBLE_EQ((double)s1, stat.max());

  stat.add_sample(s4);

  ASSERT_DOUBLE_EQ((double)27750, stat.mean());
  EXPECT_NEAR((double)41894.95793052, stat.stddev(), 1e-5);
  ASSERT_DOUBLE_EQ((double)s4, stat.min());
  ASSERT_DOUBLE_EQ((double)s1, stat.max());
}