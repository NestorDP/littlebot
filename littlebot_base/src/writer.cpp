// Copyright Nestor 2022-2023

#include <chrono>  // NOLINT
#include <iostream>
#include <memory>
#include <utility>

#include "example_interfaces/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "littlebot_base/writer.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace littlebot_base {
  Writer::Writer(const rclcpp::NodeOptions & options)
    : Node("", options)
    {}


  Writer::Writer(const rclcpp::NodeOptions & options, serial::Serial *serial)
    : Node("robot", "littlebot_base", options), serial_(serial) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto vel_right_callback =
      [this](example_interfaces::msg::Float32::ConstSharedPtr vel) -> void {
        right_vel_ = vel->data;
        msg_protocol_ << "W" << left_vel_ << "#" << right_vel_ << "#";
        send_msg_ = msg_protocol_.str();
        this->serial_->SendMsg(&send_msg_);
      };

    auto vel_left_callback =
      [this](example_interfaces::msg::Float32::ConstSharedPtr vel) -> void {
        left_vel_ = vel->data;
        msg_protocol_ << "W" << left_vel_ << "#" << right_vel_ << "#";
        send_msg_ = msg_protocol_.str();
        this->serial_->SendMsg(&send_msg_);
      };

    cmd_right_vel_sub_ =
      create_subscription<example_interfaces::msg::Float32>("cmd_right_vel",
        10, vel_right_callback);

    cmd_left_vel_sub_ =
      create_subscription<example_interfaces::msg::Float32>("cmd_left_vel",
        10, vel_left_callback);

    pub1_ = create_publisher<example_interfaces::msg::Float32>("vel_left", 10);
    pub2_ = create_publisher<example_interfaces::msg::Float32>("vel_right", 10);
    pub3_ = create_publisher<example_interfaces::msg::Float32>("pos_left", 10);
    pub4_ = create_publisher<example_interfaces::msg::Float32>("pos_right", 10);

    timer_writer_ =
      create_wall_timer(1000ms, std::bind(&Writer::writerTimer, this));
  }

  void Writer::writerTimer() {
    // msg_protocol << left_vel_ << "#" << right_vel_ << "#";
    // std::string send_msg = msg_protocol.str();
    // this->serial_->SendMsg(&send_msg);
  }

}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Writer)



