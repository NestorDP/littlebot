// Copyright 2022 Nestor

#ifndef LITTLEBOT_BASE_READER_HPP_
#define LITTLEBOT_BASE_READER_HPP_

#include <string>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "example_interfaces/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include <libserial/serial.hpp>

#include "littlebot_base/protocol.hpp"

#define LITTLEBOT_BASE_READER_PUBLIC __attribute__((visibility("default")))

namespace littlebot_base {
  class Reader : public rclcpp::Node
  {
  public:
    LITTLEBOT_BASE_READER_PUBLIC
    explicit Reader(const rclcpp::NodeOptions &options);

    LITTLEBOT_BASE_READER_PUBLIC
    explicit Reader(const rclcpp::NodeOptions & options, serial::Serial *serial);

  protected:
    void on_timer();

  private:
    size_t count_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub1_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub2_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub3_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr pub4_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string message_;
    serial::Serial *serial_;
  };

} // namespace littlebot_base

#endif // LITTLEBOT_BASE_READER_HPP_
