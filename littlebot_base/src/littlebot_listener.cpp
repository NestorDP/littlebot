// Copyright Nestor 2022

#include "littlebot_base/littlebot_listener.hpp"

namespace littlebot_base {

Listener::Listener(const rclcpp::NodeOptions & options)
    : Node("", options) 
    {}

Listener::Listener(const rclcpp::NodeOptions & options, serial::Serial *serial)
  : Node("setVelocite", options), serial_(serial){
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto callback =
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void {
        std::string send_msg = msg->data.c_str();
        serial_->SendMsg(&send_msg);
      };

    sub_ =
      create_subscription<std_msgs::msg::String>("velocidade", 10, callback);
  }
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Listener)
