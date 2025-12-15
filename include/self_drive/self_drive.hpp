#ifndef SELF_DRIVE_HPP
#define SELF_DRIVE_HPP

#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SelfDrive : public rclcpp::Node
{
public:
  SelfDrive();

private:
  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  float get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;

  const float TARGET_SPEED = 0.15;
  const float STOP_DISTANCE = 0.22;
  const float DEFAULT_RANGE = 0.4;
  const float KP = 1.2;
};
#endif
