#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#define LITTLEBOT_BASE_CPP_PUBLIC __attribute__ ((visibility("default")))
#define LITTLEBOT_BASE_CPP_LOCAL __attribute__ ((visibility("hidden")))

namespace demo_nodes_cpp {
class Listener : public rclcpp::Node {
 public:
    LITTLEBOT_BASE_CPP_PUBLIC
    explicit Listener(const rclcpp::NodeOptions & options)
    : Node("listener", options) {

      setvbuf(stdout, NULL, _IONBF, BUFSIZ);

      auto callback =
        [this](std_msgs::msg::String::ConstSharedPtr msg) -> void {
          RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        };

      sub_ =
        create_subscription<std_msgs::msg::String>("chatter", 10, callback);
    }

 private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Listener)
