// Copyright 2022 Nestor

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "littlebot_base/littlebot_talker.hpp"

using namespace std::chrono_literals;

namespace littlebot_base {

// Talker::Talker(const rclcpp::NodeOptions & options)
//     : Node("", options) 
//     {}

Talker::Talker(const rclcpp::NodeOptions & options)
: Node("talker", options), count_(0) {
  serial_.OpenPort("/dev/rfcomm0");
  serial_.SetCanonicalMode(serial::CanonicalMode::Enable);
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  timer_ = create_wall_timer(100ms, std::bind(&Talker::on_timer, this));
}


void Talker::on_timer() {
  auto msg = std::make_unique<std_msgs::msg::String>();

  serial_.ReceiveMsg(&message_);

  msg->data = message_;

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);

  pub_->publish(std::move(msg));
}
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Talker)
