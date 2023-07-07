// Copyright Nestor 2022-2023


#include "littlebot_teleop/teleop.hpp"

using namespace std::chrono_literals;

namespace littlebot_teleop {
  Teleop::Teleop(const rclcpp::NodeOptions & options)
    : Node("littlebot_teleop","", options){
      
    auto joy_callback =
      [this](sensor_msgs::msg::Joy::ConstSharedPtr joy) -> void {
        z_joy_value_ = joy->axes[0];
        x_joy_value_ = joy->axes[3];
      };
    
    sub_joy_ =
      create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    pub_twist_ = 
      this->create_publisher<geometry_msgs::msg::Twist>(
      "joy_teleop/cmd_vel",
      10);


    timer_ = create_wall_timer(100ms, std::bind(&Teleop::teleopPublisher, this));
  }

  void Teleop::teleopPublisher() {
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = x_joy_value_;
  msg->angular.z = z_joy_value_;
  pub_twist_->publish(std::move(msg));
  }

}  // namespace littlebot_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_teleop::Teleop) 
