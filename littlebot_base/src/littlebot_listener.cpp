// Copyright Nestor 2022

#include "littlebot_base/littlebot_listener.hpp"

namespace littlebot_base {

Listener::Listener(const rclcpp::NodeOptions & options)
  : Node("listener", options), s_("/dev/pts/5"){

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto callback =
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        s_.SetVelocity(3.14, 3.14);
        s_.LittlebotWrite();
      };

    sub_ =
      create_subscription<std_msgs::msg::String>("chatter", 10, callback);
  }
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Listener)
