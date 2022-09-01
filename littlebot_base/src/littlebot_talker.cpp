
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
class Listener : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
      };
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

} 
RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Listener)
