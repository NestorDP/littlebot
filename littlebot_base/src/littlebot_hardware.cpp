#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "littlebot_base/littlebot_hardware.hpp"

namespace littlebot_base
{
hardware_interface::return_type LittlebotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];
  serial_port_ = info_.hardware_parameters["device"];
  encoder_ppr = std::stoi(info_.hardware_parameters["encoder_PPR"]);

  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Port: %s", serial_port_.c_str());

  serial_device_.OpenPort(serial_port_);
  serial_device_.SetFlowControl(serial::FlowControl::Software);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Littlebot has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotHardware"),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}


std::vector<hardware_interface::StateInterface> LittlebotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_POSITION, &left_position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_POSITION, &right_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &left_velocitie_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &right_velocitie_));

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> LittlebotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &left_command_velocity_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &right_command_velocity_));

  return command_interfaces;
}


hardware_interface::return_type LittlebotHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Starting ...please wait...");

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LittlebotHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Stopping ...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LittlebotHardware::read()
{
  std::size_t split_character;
  std::size_t begin_character;
  std::size_t end_character;
  std::string final_mgs;
  std::string med_msg;
  do {
    serial_device_.ReceiveMsg(&message_protocol_); // "<left_vel#right_vel#left_pos#right_pos#>"
    begin_character = message_protocol_.find("<");
    end_character = message_protocol_.find(">");
  } while (begin_character != 0 || end_character == std::string::npos);

  message_protocol_.erase(0, 1);
  final_mgs = message_protocol_.substr(0, end_character); 

  split_character = final_mgs.find("#");
  med_msg = final_mgs.substr(0, split_character);
  final_mgs.erase(0, split_character + 1);
  left_velocitie_ = stoi(med_msg);

  split_character = final_mgs.find("#");
  med_msg = final_mgs.substr(0, split_character);
  final_mgs.erase(0, split_character + 1);
  right_velocitie_ = stoi(med_msg);

  split_character = final_mgs.find("#");
  med_msg = final_mgs.substr(0, split_character);
  final_mgs.erase(0, split_character + 1);
  left_position_ = stoi(med_msg);

  split_character = final_mgs.find("#");
  med_msg = final_mgs.substr(0, split_character);
  final_mgs.erase(0, split_character + 1);
  right_position_ = stoi(med_msg);

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LittlebotHardware::write()
{
  std::stringstream msg_protocol;

  msg_protocol << left_wheel_name_ << "#" << right_wheel_name_  << "#"; 
  std::string send_msg = msg_protocol.str();
  serial_device_.SendMsg(&send_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  littlebot_base::LittlebotHardware, hardware_interface::SystemInterface)