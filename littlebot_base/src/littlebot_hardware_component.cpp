// @ Copyright 2024-2025 Nestor Neto
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "littlebot_base/littlebot_hardware_component.hpp"

namespace littlebot_base
{
hardware_interface::CallbackReturn LittlebotHardwareComponent::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  hw_status_positions_.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_status_velocities_.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != kNumCommandInterface_) {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != kNumStateInterface_) {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::cout << "###" << joint.name << " command interfaces: "
              << joint.command_interfaces[0].name << ", state interfaces: "
              << joint.state_interfaces[0].name << ", " << joint.state_interfaces[1].name
              << std::endl;
  }

  // Get parameters from the hardware info
  if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end()) {
    serial_port_name_ = info_.hardware_parameters.at("serial_port");
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("LittlebotSystemHardware"),
      "'serial_port' parameter not found in the hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end()) {
    try {
      serial_baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("LittlebotSystemHardware"),
        "Failed to parse 'baudrate' parameter: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("LittlebotSystemHardware"),
      "'baudrate' parameter not found in the hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout << "### Serial port: " << serial_port_name_
            << ", Baudrate: " << serial_baudrate_ << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto serial_port = std::make_shared<SerialPort>();
  this->setupDriver(serial_port, serial_port_name_, serial_baudrate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_status_positions_.size(); i++) {
    if (std::isnan(hw_status_positions_[i])) {
      hw_status_positions_[i] = 0;
      hw_status_velocities_[i] = 0;
      hw_commands_velocities_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("LittlebotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LittlebotHardwareComponent::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(rclcpp::get_logger("LittlebotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LittlebotHardwareComponent::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_status_positions_[i]);
    std::cout << "### Exporting state interface for joint: " << info_.joints[i].name
              << " position: " << hw_status_positions_[i] << std::endl;
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_status_velocities_[i]);
    std::cout << "### Exporting state interface for joint: " << info_.joints[i].name
              << " velocity: " << hw_status_velocities_[i] << std::endl;
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LittlebotHardwareComponent::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]);
    std::cout << "### Exporting command interface for joint: " << info_.joints[i].name
              << " velocity: " << hw_commands_velocities_[i] << std::endl;
  }
  return command_interfaces;
}

hardware_interface::return_type LittlebotHardwareComponent::read(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  littlebot_driver_->sendData('S');
  littlebot_driver_->receiveData();
  auto map_status_positions = littlebot_driver_->getStatusPositions();
  auto map_status_velocities = littlebot_driver_->getStatusVelocities();

  for (const auto & name : littlebot_driver_->getJointNames()) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (info_.joints[i].name == name) {
        if (map_status_positions.find(name) != map_status_positions.end()) {
          hw_status_positions_[i] = map_status_positions.at(name);
        }
        if (map_status_velocities.find(name) != map_status_velocities.end()) {
          hw_status_velocities_[i] = map_status_velocities.at(name);
        }
      }
    }
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LittlebotHardwareComponent::write(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  std::map<std::string, float> map_command_velocities;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    map_command_velocities[info_.joints[i].name] = hw_commands_velocities_[i];
  }
  littlebot_driver_->setCommandVelocities(map_command_velocities);
  littlebot_driver_->sendData('C');
  return hardware_interface::return_type::OK;
}

void LittlebotHardwareComponent::setupDriver(
  std::shared_ptr<littlebot_base::ISerialPort> serial_port,
  const std::string & port, int baudrate)
{
  littlebot_driver_ = std::make_shared<littlebot_base::LittlebotDriver>(
    serial_port, port, baudrate);

  std::vector<std::string> joint_names{info_.joints[0].name, info_.joints[1].name};
  littlebot_driver_->setJointNames(joint_names);
}

}   //  namespace littlebot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  littlebot_base::LittlebotHardwareComponent,
  hardware_interface::SystemInterface)
