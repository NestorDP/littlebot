// Copyright Nestor 2022-2023


#include "littlebot_teleop/teleop.hpp"

using namespace std::chrono_literals;

namespace littlebot_teleop {
  Teleop::Teleop(const rclcpp::NodeOptions & options)
    : Node("littlebot_teleop", options){
      
    auto joy_callback =
      [this](sensor_msgs::msg::Joy::ConstSharedPtr joy) -> void {
        z_joy_value_ = joy->axes[0];
        x_joy_value_ = joy->axes[3];
      };
    
    sub_joy_ =
      create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    //pub1_ = create_publisher<example_interfaces::msg::Float32>("vel_left", 10);


    //timer_writer_ = create_wall_timer(1000ms, std::bind(&Writer::writerTimer, this));
  }

  // void Writer::writerTimer() {
    
  //   // msg_protocol << left_vel_ << "#" << right_vel_ << "#"; 
  //   // std::string send_msg = msg_protocol.str();
  //   // this->serial_->SendMsg(&send_msg);
  // }

}  // namespace littlebot_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(littlebot_teleop::Teleop) 
