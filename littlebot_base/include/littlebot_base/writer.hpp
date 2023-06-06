
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
    void writerTimer();

   private:
    /* Command right velocity subscriber*/
    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_right_vel_sub_;
    /* Command right velocity subscriber*/
    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_left_vel_sub_;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub1_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub2_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub3_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub4_;

    std::stringstream msg_protocol_;
    std::string send_msg_;

    /*Right velocite variable*/
    float right_vel_;
    /*Left velocite variable*/
    float left_vel_;
    /*Serial port object*/
    serial::Serial *serial_;
    /* */
    rclcpp::TimerBase::SharedPtr timer_writer_;
  };
}  // namespace littlebot_base

#endif  // LITTLEBOT_BASE_WRITER_HPP_
