// Copyright 2023 Nestor Neto

#ifndef LITTLEBOT_TELEOP_INCLUDE_LITTLEBOT_TELEOP_TELEOP_HPP_
#define LITTLEBOT_TELEOP_INCLUDE_LITTLEBOT_TELEOP_TELEOP_HPP_

#include <string>
#include <sstream>
#include <chrono>  // NOLINT
#include <iostream>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define LITTLEBOT_TELEOP_CPP_PUBLIC __attribute__ ((visibility("default")))

namespace littlebot_teleop {
class Teleop : public rclcpp::Node {
 public:
  LITTLEBOT_TELEOP_CPP_PUBLIC
  explicit Teleop(const rclcpp::NodeOptions & options);

 protected:
  void teleopPublisher(void);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;

  float x_joy_value_;
  float z_joy_value_;
};
}  // namespace littlebot_teleop

#endif  // LITTLEBOT_TELEOP_INCLUDE_LITTLEBOT_TELEOP_TELEOP_HPP_/
