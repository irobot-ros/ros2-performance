
#include <gtest/gtest.h>

#include "performance_test/ros2/tracker.hpp"



int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



TEST(TrackerTest, TrackerInitTest)
{
  performance_test::Tracker tracker("", "", performance_test::Tracker::TrackingOptions());

  ASSERT_EQ((unsigned long int)0, tracker.lost());
  ASSERT_EQ((unsigned long int)0, tracker.late());
  ASSERT_EQ((unsigned long int)0, tracker.too_late());
  ASSERT_EQ((unsigned long int)0, tracker.received());
  ASSERT_EQ((unsigned long int)0, tracker.last());

  EXPECT_TRUE(std::isnan(tracker.stat().mean()));
  EXPECT_TRUE(std::isnan(tracker.stat().stddev()));
  EXPECT_TRUE(std::isnan(tracker.stat().min()));
  EXPECT_TRUE(std::isnan(tracker.stat().max()));
  ASSERT_EQ((unsigned long int)0, tracker.stat().n());
}


TEST(TrackerTest, TrackerScanTest)
{
  // tracker with disabled tracking options
  performance_test::Tracker tracker("", "", performance_test::Tracker::TrackingOptions(false));

  auto header = performance_test_msgs::msg::PerformanceHeader();
  header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

  rclcpp::Time t_now1(0, 10, RCL_ROS_TIME);
  tracker.scan(header, t_now1, nullptr);

  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(10), tracker.stat().mean());
  ASSERT_DOUBLE_EQ((double)0, tracker.stat().stddev());
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(10), tracker.stat().min());
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(10), tracker.stat().max());
  ASSERT_EQ((unsigned long int)RCL_NS_TO_US(10), tracker.last());

  rclcpp::Time t_now2(0, 200, RCL_ROS_TIME);
  tracker.scan(header, t_now2, nullptr);

  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(105), tracker.stat().mean());
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(95), tracker.stat().stddev());
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(10), tracker.stat().min());
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(200), tracker.stat().max());
  ASSERT_EQ((unsigned long int)RCL_NS_TO_US(200), tracker.last());

  // This is 1e9 nanoseconds
  rclcpp::Time t_now3(1, 0, RCL_ROS_TIME);
  tracker.scan(header, t_now3, nullptr);

  EXPECT_NEAR((double)RCL_NS_TO_US(333333333.33333331), tracker.stat().mean(), 1e-1);
  EXPECT_NEAR((double)RCL_NS_TO_US(471404471.29356), tracker.stat().stddev(), 1e-1);
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(10), tracker.stat().min());
  ASSERT_DOUBLE_EQ((double)RCL_NS_TO_US(1e9), tracker.stat().max());
  ASSERT_EQ((unsigned long int)RCL_NS_TO_US(1e9),tracker.last());
}


TEST(TrackerTest, TrackingOptionsTest)
{
  performance_test::Tracker::TrackingOptions t_options;
  t_options.late_percentage = 20;
  t_options.late_absolute_us = 5000;
  t_options.too_late_percentage = 100;
  t_options.too_late_absolute_us = 50000;

  performance_test::Tracker tracker("", "", t_options);

  auto header = performance_test_msgs::msg::PerformanceHeader();
  header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  header.frequency = 100;

  // this message is on time
  rclcpp::Time t_now1(0, 1, RCL_ROS_TIME);
  tracker.scan(header, t_now1, nullptr);

  ASSERT_EQ((unsigned long int)0, tracker.late());
  ASSERT_EQ((unsigned long int)0, tracker.too_late());

  // this message is late because it exceeds late_percentage
  rclcpp::Time t_now2(0, 3e6, RCL_ROS_TIME);
  tracker.scan(header, t_now2, nullptr);

  ASSERT_EQ((unsigned long int)1,tracker.late());
  ASSERT_EQ((unsigned long int)0, tracker.too_late());

  // this message is late because it exceeds late_absolute_us
  rclcpp::Time t_now3(0, 6e6, RCL_ROS_TIME);
  tracker.scan(header, t_now3, nullptr);

  ASSERT_EQ((unsigned long int)2, tracker.late());
  ASSERT_EQ((unsigned long int)0, tracker.too_late());

  // store mean as it should not be updated with too late messages
  double mean = tracker.stat().mean();

  // this message is too late because it exceeds too_late_percentage
  rclcpp::Time t_now4(0, 11e6, RCL_ROS_TIME);
  tracker.scan(header, t_now4, nullptr);

  ASSERT_EQ((unsigned long int)2, tracker.late());
  ASSERT_EQ((unsigned long int)1, tracker.too_late());
  ASSERT_DOUBLE_EQ(mean, tracker.stat().mean());

  // this message is too late because it exceeds too_late_absolute_us
  rclcpp::Time t_now5(0, 111e6, RCL_ROS_TIME);
  tracker.scan(header, t_now5, nullptr);

  ASSERT_EQ((unsigned long int)2, tracker.late());
  ASSERT_EQ((unsigned long int)2, tracker.too_late());
  ASSERT_DOUBLE_EQ(mean, tracker.stat().mean());
}
