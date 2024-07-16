// //  @ Copyright 2023 Nestor Neto

#include <rclcpp/rclcpp.hpp>

#include "littlebot_base/littlebot_hardware.hpp"

namespace littlebot_base
{

LittlebotHardwareComponent::~LittlebotHardwareComponent()
{

}

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
    // LittlebotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
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

    if (joint.state_interfaces.size() != 2)
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

  for (ManipulatorJointData & joint : joints_) {
    for (auto & state_interface : joint.state_interfaces) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.name, state_interface.first, &state_interface.second));
    }
  }

  for (const std::string & sensor_state_interface_name : sensor_state_interfaces_in_use_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor_name_, sensor_state_interface_name,
        &force_torque_state_interfaces_map_[sensor_state_interface_name]));
  }
  return state_interfaces;
}

hardware_interface::return_type LittlebotHardwareComponent::read(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  for (const std::string & state_interface_name : all_joint_state_interface_names_) {
    for (const types::DeviceId & joint_id : state_interfaces_map_[state_interface_name]) {
      uint8_t joint_index = (static_cast<uint8_t>(joint_id) - 1);

      try {
        if (state_interface_name == "voltage") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getVoltageInVolts(joint_id);
        }
        if (state_interface_name == "velocity") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getVelocityInRadiansPerSecond(joint_id);
        }
        if (state_interface_name == "position") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getPositionInRadians(joint_id);
        }
        if (state_interface_name == "relative_position") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getRelativePositionInRadians(joint_id);
        }
        if (state_interface_name == "current") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getCurrentInMiliampere(joint_id);
        }
        if (state_interface_name == "internal_humidity") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getInternalHumidityPercent(joint_id);
        }
        if (state_interface_name == "internal_temperature") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getInternalTemperatureInDegreesCelsius(joint_id);
        }
        if (state_interface_name == "internal_pressure") {
          joints_[joint_index].state_interfaces[state_interface_name] =
            manipulator_communication_ptr_->getInternalPressureInBar(joint_id);
        }
      } catch (const driver_base::TimeoutError & e) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
            hardware_component_name_),
          "Timeout error while reading the  " << state_interface_name << "of the manipulator joints: " <<
            e.what());

        joints_[joint_index].state_interfaces[state_interface_name] =
          std::numeric_limits<double>::quiet_NaN();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
            hardware_component_name_),
          "Error while reading the  " << state_interface_name << "of the manipulator joints: " <<
            e.what());

        joints_[joint_index].state_interfaces[state_interface_name] =
          std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  try {
    types::ForceTorque sensor_reading =
      manipulator_communication_ptr_->getForceTorqueReading();

    force_torque_state_interfaces_map_[all_sensor_state_interface_names_[0]] =
      sensor_reading.force_x;
    force_torque_state_interfaces_map_[all_sensor_state_interface_names_[1]] =
      sensor_reading.force_y;
    force_torque_state_interfaces_map_[all_sensor_state_interface_names_[2]] =
      sensor_reading.force_z;
    force_torque_state_interfaces_map_[all_sensor_state_interface_names_[3]] =
      sensor_reading.torque_x;
    force_torque_state_interfaces_map_[all_sensor_state_interface_names_[4]] =
      sensor_reading.torque_y;
    force_torque_state_interfaces_map_[all_sensor_state_interface_names_[5]] =
      sensor_reading.torque_z;
  } catch (const driver_base::TimeoutError & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        hardware_component_name_),
      "Timeout error while reading the Force Torque sensor: %s", e.what());

    for (auto & sensor_state_interface : force_torque_state_interfaces_map_) {
      sensor_state_interface.second = std::numeric_limits<float>::quiet_NaN();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        hardware_component_name_),
      "An error occurred while reading the Force Torque sensor: %s", e.what());

    for (auto & sensor_state_interface : force_torque_state_interfaces_map_) {
      sensor_state_interface.second = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> LittlebotHardwareComponent::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  return command_interfaces;
}

hardware_interface::return_type LittlebotHardwareComponent::write(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}

void LittlebotHardwareComponent::configureLimitsOfJoints()
{
  for (const std::string & configuration_name : configuration_parameters_names_) {
    for (const types::DeviceId & joint_id : configurations_map_[configuration_name]) {
      if (configuration_name == "maximum_position") {
        continue;   // The maximum position is configured in the same time as the minimum position.
      }

      float minimum_limit =
        joints_[static_cast<uint8_t>(joint_id) - 1].limit_values[configuration_name].first;

      float maximum_limit =
        joints_[static_cast<uint8_t>(joint_id) - 1].limit_values[configuration_name].second;

      manipulator_communication_ptr_->setJointLimits(
        configurations_functions_map_[configuration_name], minimum_limit, maximum_limit,
        joint_id);
    }
  }
}

void LittlebotHardwareComponent::fillForceTorqueStateInterfacesMapFromUrdf(
  const std::vector<hardware_interface::ComponentInfo> & sensors)
{
  if (sensors.size() != kSensorsInManipulator || sensors[0].name != sensor_name_) {
    throw std::invalid_argument(
            "The BravoSevenHardware component should include the configured Force Torque sensor.");
  }

  for (const hardware_interface::InterfaceInfo & state_interface : sensors[0].state_interfaces) {
    auto vector_iterator = std::find(
      all_sensor_state_interface_names_.begin(),
      all_sensor_state_interface_names_.end(), state_interface.name);

    if (vector_iterator == all_sensor_state_interface_names_.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          hardware_component_name_),
        "The %s state interface will not be configured, because it is not listed on the BravoSevenHardware.",
        state_interface.name.c_str());
    } else {
      sensor_state_interfaces_in_use_.push_back(state_interface.name);
      force_torque_state_interfaces_map_[state_interface.name] = 0.0;
    }
  }
}

void LittlebotHardwareComponent::fillJointsConfigurationMapFromUrdf(
  const std::vector<hardware_interface::ComponentInfo> & joints)
{
  if (joints.size() != types::kJointsInManipulator) {
    throw std::invalid_argument(
            "The BravoSevenHardware component must have eight joint interfaces.");
  }

  for (const hardware_interface::ComponentInfo & joint_info : joints) {
    ManipulatorJointData valid_joint;
    if (types::bravo_seven_devices_map_.find(joint_info.name) ==
      types::bravo_seven_devices_map_.end())
    {
      throw std::invalid_argument(
              "The joint " + joint_info.name +
              " has an invalid name. The joints of the bravo seven Manipulator are: rotate_base, bend_shoulder,"
              " bend_elbow, rotate_elbow, bend_forearm, rotate_end_effector and end_effector.");
    }

    for (const ManipulatorJointData & joint : joints_) {
      if (joint_info.name == joint.name) {
        throw std::invalid_argument(
                "The joint " + joint_info.name + " has already been configured.");
      }
    }

    valid_joint.name = joint_info.name;

    if (!this->areJointParametersValid(valid_joint)) {
      throw std::invalid_argument(
              "The joint " + joint_info.name + "  has invalid parameters, check the values. "
      );
    }

    this->getJointStateInterfaces(joint_info.state_interfaces, valid_joint);

    joints_.at(static_cast<uint8_t>(valid_joint.id) - 1) = valid_joint;
  }
}

bool LittlebotHardwareComponent::areJointParametersValid(
  ManipulatorJointData & valid_joint)
{
  valid_joint.id = types::bravo_seven_devices_map_.at(valid_joint.name);

  for (const std::string & parameter_name : configuration_parameters_names_) {
    std::string parameter_as_str = hardware_description_helper_ptr_->getComponentParameter(
      valid_joint.name, parameter_name);

    if (!parameter_as_str.empty()) {
      float parameter_as_float;
      try {
        parameter_as_float = std::stof(parameter_as_str);
      } catch (const std::invalid_argument & e) {
        return false;
      }

      float minimum_limit;
      float maximum_limit;

      if (parameter_name == "minimum_position" || parameter_name == "maximum_position") {
        try {
          this->getJointPositionLimitsParameters(
            valid_joint.name, minimum_limit, maximum_limit);
        } catch (const std::invalid_argument & e) {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(hardware_component_name_),
            "Error while reading the parameters of the joint " << valid_joint.name << ": " <<
              e.what());

          return false;
        }
      } else {
        minimum_limit = -parameter_as_float;
        maximum_limit = parameter_as_float;
      }
      valid_joint.limit_values[parameter_name] = std::make_pair(minimum_limit, maximum_limit);

      configurations_map_[parameter_name].push_back(valid_joint.id);
    }
  }
  return true;
}

void LittlebotHardwareComponent::getJointPositionLimitsParameters(
  std::string valid_joint_name, float & minimum_position, float & maximum_position)
{
  std::string maximum_position_as_str =
    hardware_description_helper_ptr_->getComponentParameter(
    valid_joint_name, "maximum_position");

  std::string minimum_position_as_str =
    hardware_description_helper_ptr_->getComponentParameter(
    valid_joint_name, "minimum_position");

  if (maximum_position_as_str.empty()) {
    throw std::invalid_argument(
            "The parameter maximum_position cannot be empty.");
  }

  if (minimum_position_as_str.empty()) {
    throw std::invalid_argument(
            "The parameter minimum_position cannot be empty.");
  }

  try {
    minimum_position = std::stof(minimum_position_as_str);
    maximum_position = std::stof(maximum_position_as_str);
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument(
            "The position limits contain invalid values.");
  }
}

void LittlebotHardwareComponent::getJointStateInterfaces(
  const std::vector<hardware_interface::InterfaceInfo> & state_interfaces,
  ManipulatorJointData & valid_joint)
{
  if (!state_interfaces.empty()) {
    for (const hardware_interface::InterfaceInfo & state_interface : state_interfaces) {
      auto vector_iterator = std::find(
        all_joint_state_interface_names_.begin(),
        all_joint_state_interface_names_.end(), state_interface.name);

      if (vector_iterator == all_joint_state_interface_names_.end()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            hardware_component_name_),
          "The %s state interface will not be configured, because it is not listed on the BravoSevenHardware.",
          state_interface.name.c_str());
      } else {
        state_interfaces_map_[state_interface.name].push_back(valid_joint.id);
        valid_joint.state_interfaces[state_interface.name] = 0;
      }
    }
  }
}

void LittlebotHardwareComponent::setBravoSevenCommunication(
  std::shared_ptr<BravoSevenCommunicationInterface> ptr)
{
  manipulator_communication_ptr_ = ptr;
}
}   //  namespace littlebot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  littlebot_base::LittlebotHardwareComponent,
  hardware_interface::SystemInterface)

