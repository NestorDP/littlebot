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
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  serial_port_ = info_.hardware_parameters["serial_port"];

  // RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Port: %s", serial_port_.c_str());

  //TODO: connect to serial port

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
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LittlebotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type LittlebotHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

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
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Reading...");

  double radius = 0.02;  // radius of the wheels
  double dist_w = 0.1;   // distance between the wheels
  double dt = 0.01;      // Control period
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[1] + dt * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("LittlebotHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * dt;
  base_y_ += base_dy * dt;
  base_theta_ += base_dtheta * dt;

  RCLCPP_INFO(
    rclcpp::get_logger("LittlebotHardware"), "Joints successfully read! (%.5f,%.5f,%.5f)",
    base_x_, base_y_, base_theta_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LittlebotHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("LittlebotHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  littlebot_base::LittlebotHardware, hardware_interface::SystemInterface)