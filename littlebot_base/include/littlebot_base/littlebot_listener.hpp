
// Copyright Nestor 2022

#ifndef LITTLEBOT_BASE_LITTLEBOT_LISTENER_HPP_
#define LITTLEBOT_BASE_LITTLEBOT_LISTENER_HPP_

#include <string>

#include <libserial/serial.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "littlebot_base/littlebot_communication_protocol.hpp"

#define LITTLEBOT_LISTENER_CPP_PUBLIC __attribute__ ((visibility("default")))
#define LITTLEBOT_LISTENER_CPP_LOCAL __attribute__ ((visibility("hidden")))

namespace littlebot_base {
class Listener : public rclcpp::Node {
 public:
   LITTLEBOT_LISTENER_CPP_PUBLIC
   explicit Listener(const rclcpp::NodeOptions & options);

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  comm::LittlebotCommunicationProtocol s_;
  std::shared_ptr<std::string> send_ptr_;
  std::string port_ = "/dev/pts/5";
};

}  // namespace littlebot_base

#endif  // LITTLEBOT_BASE_LITTLEBOT_LISTENER_HPP_
