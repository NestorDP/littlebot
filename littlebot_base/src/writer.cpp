// Copyright Nestor 2022

#include "littlebot_base/writer.hpp"

namespace littlebot_base {
  Writer::Writer(const rclcpp::NodeOptions & options)
    : Node("", options) 
    {}

  Writer::Writer(const rclcpp::NodeOptions & options, serial::Serial *serial)
    : Node("writer","littlebot_base", options), serial_(serial){
      
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto callback =
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void {
        std::stringstream msg_protocol;
        int a = -20000;
        int b = 20000;
        msg_protocol << a << "#" << b << "#"; 
        std::string send_msg = msg_protocol.str();
        serial_->SendMsg(&send_msg);
      };

    sub_ =
      create_subscription<std_msgs::msg::String>("velocidade", 10, callback);
  }
}  // namespace littlebot_base

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_base::Writer)
