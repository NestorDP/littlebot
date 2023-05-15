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
  : Node("reader", "littlebot_base", options), count_(0), serial_(serial) {

    pub_ = create_publisher<std_msgs::msg::String>("velocite_feedback", 10);
    timer_ = create_wall_timer(100ms, std::bind(&Reader::on_timer, this));
  }


  void Reader::on_timer() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    std::size_t found;
    std::string final_mgs;

    do {
      serial_->ReceiveMsg(&message_);
      found = message_.find("<");
    } while (found != 0);

    message_.erase(0, 1);
    final_mgs = message_.substr(0, message_.find(">"));

    msg->data = final_mgs;
    pub_->publish(std::move(msg));
  }
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Reader)
