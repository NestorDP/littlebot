// Copyright Nestor 2022

#ifndef LITTLEBOT_BASE_LITTLEBOT_LISTENER_HPP_
#define LITTLEBOT_BASE_LITTLEBOT_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#define LITTLEBOT_BASE_CPP_PUBLIC __attribute__ ((visibility("default")))
#define LITTLEBOT_BASE_CPP_LOCAL __attribute__ ((visibility("hidden")))

namespace littlebot_listener {
class Listener : public rclcpp::Node {
 public:
    LITTLEBOT_BASE_CPP_PUBLIC
    explicit Listener(const rclcpp::NodeOptions & options);

 private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace littlebot_listener

#endif  // LITTLEBOT_BASE_LITTLEBOT_LISTENER_HPP_
