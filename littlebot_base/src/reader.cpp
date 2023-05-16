// Copyright 2022 Nestor

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "example_interfaces/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "littlebot_base/reader.hpp"

using namespace std::chrono_literals;

namespace littlebot_base {
  Reader::Reader(const rclcpp::NodeOptions & options)
      : Node("", options) 
      {}


  Reader::Reader(const rclcpp::NodeOptions & options, serial::Serial *serial)
  : Node("reader", "littlebot_base", options), count_(0), serial_(serial) {

    pub1_ = create_publisher<example_interfaces::msg::Float32>("vel_left", 10);
    pub2_ = create_publisher<example_interfaces::msg::Float32>("vel_right", 10);
    pub3_ = create_publisher<example_interfaces::msg::Float32>("pos_left", 10);
    pub4_ = create_publisher<example_interfaces::msg::Float32>("pos_right", 10);
    timer_ = create_wall_timer(100ms, std::bind(&Reader::on_timer, this));
  }


  void Reader::on_timer() {
    auto vel_left_topic = std::make_unique<example_interfaces::msg::Float32>();
    auto vel_right_topic = std::make_unique<example_interfaces::msg::Float32>();
    auto pos_left_topic = std::make_unique<example_interfaces::msg::Float32>();
    auto pos_right_topic = std::make_unique<example_interfaces::msg::Float32>();

    std::size_t found;
    std::size_t found_begin_char;
    std::size_t found_end_char;
    std::string final_mgs;
    std::string med_msg;

    do {
      serial_->ReceiveMsg(&message_); // "<left_vel#right_vel#left_pos#right_pos#>"
      found_begin_char = message_.find("<");
      found_end_char = message_.find(">");
    } while (found_begin_char != 0 || found_end_char == std::string::npos);

    message_.erase(0, 1);
    final_mgs = message_.substr(0, message_.find(">")); 


    found = final_mgs.find("#");
    med_msg = final_mgs.substr(0, found);
    final_mgs.erase(0, found + 1);
    vel_left_topic->data = stoi(med_msg);

    found = final_mgs.find("#");
    med_msg = final_mgs.substr(0, found);
    final_mgs.erase(0, found + 1);
    vel_right_topic->data = stoi(med_msg);

    found = final_mgs.find("#");
    med_msg = final_mgs.substr(0, found);
    final_mgs.erase(0, found + 1);
    pos_left_topic->data = stoi(med_msg);

    found = final_mgs.find("#");
    med_msg = final_mgs.substr(0, found);
    final_mgs.erase(0, found + 1);
    pos_right_topic->data = stoi(med_msg);

    pub1_->publish(std::move(vel_left_topic));
    pub2_->publish(std::move(vel_right_topic));
    pub3_->publish(std::move(pos_left_topic));
    pub4_->publish(std::move(pos_right_topic));
  }
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Reader)
