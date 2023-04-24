// Copyright 2022 Nestor

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "littlebot_base/reader.hpp"

using namespace std::chrono_literals;

namespace littlebot_base {
  Reader::Reader(const rclcpp::NodeOptions & options)
      : Node("", options) 
      {}


  Reader::Reader(const rclcpp::NodeOptions & options, serial::Serial *serial)
  : Node("getVelocite", options), count_(0), serial_(serial) {

    pub_ = create_publisher<std_msgs::msg::String>("feedback", 10);
    timer_ = create_wall_timer(1000ms, std::bind(&Reader::on_timer, this));
  }


  void Reader::on_timer() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    
    serial_->ReceiveMsg(&message_);
    msg->data = message_;
    pub_->publish(std::move(msg));
  }
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Reader)
