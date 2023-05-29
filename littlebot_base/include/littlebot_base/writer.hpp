
// Copyright Nestor 2022

#ifndef LITTLEBOT_BASE_WRITER_HPP_
#define LITTLEBOT_BASE_WRITER_HPP_

#include <string>
#include <sstream>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "example_interfaces/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "littlebot_base/protocol.hpp"

#define LITTLEBOT_BASE_WRITER_CPP_PUBLIC __attribute__ ((visibility("default")))
#define LITTLEBOT_BASE_WRITER_CPP_LOCAL __attribute__ ((visibility("hidden")))

namespace littlebot_base {
  class Writer : public rclcpp::Node {
   public:
    LITTLEBOT_BASE_WRITER_CPP_PUBLIC
    explicit Writer(const rclcpp::NodeOptions & options);

    LITTLEBOT_BASE_WRITER_CPP_PUBLIC
    explicit Writer(const rclcpp::NodeOptions & options, serial::Serial *serial);

   protected:
    void on_timer();

   private:
    /* Command right velocity subscriber*/
    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_right_vel_sub_;
    /* Command right velocity subscriber*/
    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_left_vel_sub_;
    /*Right velocite variable*/
    float right_vel_;
    /*Left velocite variable*/
    float left_vel_;
    /*Serial port object*/
    serial::Serial *serial_;
    /* */
    rclcpp::TimerBase::SharedPtr timer_;
  };
}  // namespace littlebot_base

#endif  // LITTLEBOT_BASE_WRITER_HPP_
