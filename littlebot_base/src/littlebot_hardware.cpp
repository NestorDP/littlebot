// //  @ Copyright 2023 Nestor Neto

#include <rclcpp/rclcpp.hpp>

#include "littlebot_base/littlebot_hardware.hpp"

namespace littlebot_base
{

// LittlebotHardwareComponent::~LittlebotHardwareComponent()
// {
// }

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
     if (joint.command_interfaces.size() != kNumCommandInterface_)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != kNumStateInterface_)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  if (manipulator_communication_ptr_ == nullptr) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        hardware_component_name_), "The Bravo Seven Communication plugin is not set.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    manipulator_communication_ptr_->openConnection(uri_, read_timeout_, write_timeout_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger(hardware_component_name_), "Error while opening connection: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    this->configureLimitsOfJoints();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger(hardware_component_name_),
      "Error while configuring the manipulator joints: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
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

  RCLCPP_INFO(rclcpp::get_logger("LittlebotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  if (manipulator_communication_ptr_->isConnectionOpen()) {
    manipulator_communication_ptr_->closeConnection();
  }

  RCLCPP_INFO(
    rclcpp::get_logger(
      hardware_component_name_), "BravoSevenHardware successfully shut down");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_error(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  if (manipulator_communication_ptr_->isConnectionOpen()) {
    manipulator_communication_ptr_->closeConnection();
  }

  RCLCPP_INFO(
    rclcpp::get_logger(
      hardware_component_name_), "BravoSevenHardware successfully recovered from error state.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LittlebotHardwareComponent::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type LittlebotHardwareComponent::read(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LittlebotHardwareComponent::write(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
   // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());

    hw_velocities_[i] = hw_commands_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}   //  namespace littlebot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  littlebot_base::LittlebotHardwareComponent,
  hardware_interface::SystemInterface)

