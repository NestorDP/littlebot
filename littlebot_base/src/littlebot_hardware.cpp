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

  commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &states_velocities_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> LittlebotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &commands_velocities_[i]));
  }

  return command_interfaces;
}


hardware_interface::return_type LittlebotHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotHardware"), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < states_positions_.size(); i++)
  {
    if (std::isnan(states_positions_[i]))
    {
      states_positions_[i] = 0;
      states_velocities_[i] = 0;
      commands_velocities_[i] = 0;
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
  //******************************************************************************
  //ler os valores dos econders, de velocidade e ângulo
  //lembrar de calcular a velocidade angular em rad/s
  //usar a comunicação serial para ler a mensagem e "separar"
  // as variáveis
  //******************************************************************************
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LittlebotHardware::write()
{
  //******************************************************************************
  //escrever valores das velocidades angular em rad/s
  //montar a mensagem a partir da variavel comands_velocities
  //enviar através da comunicação serial
  //******************************************************************************

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  littlebot_base::LittlebotHardware, hardware_interface::SystemInterface)