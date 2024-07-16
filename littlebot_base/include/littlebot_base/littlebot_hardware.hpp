// //  @ Copyright 2023 Nestor Neto

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "littlebot_base/littlebot_communication.hpp"

namespace littlebot_base
{

class LittlebotHardwareComponent : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Deconstructor for the LittlebotHardwareComponent class
   *
   */
  ~LittlebotHardwareComponent() override;

  /**
   * @brief
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief 
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 
   */
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 
  */
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief  
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief 
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief 
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  

private:
  /**
   * @brief Get the state interfaces from the Force Torque Sensor through the Urdf.
   *
   * This method checks the Force Torque state interfaces listed in Urdf and saves them
   * in the @ref all_sensor_state_interface_names_ variable.
   *
   * @param sensors The vector of sensors (ComponentInfo).
   * @throw std::invalid_argument If the Force Torque is not listed.
   */
  void fillForceTorqueStateInterfacesMapFromUrdf(
    const std::vector<hardware_interface::ComponentInfo> & sensors);

  /**
   * @brief Get the Joints passed through the Urdf.
   *
   * This method calls @ref areJointParametersValid and @ref getJointStateInterfaces to check the parameters and map the state interfaces to each joint, respectively. When the joints are correctly configured, they are saved in the vector @ref joints_.
   *
   * @param joints The vector of joints (ComponentInfo).
   * @throw std::invalid_argument If there are not eight joints correctly named.
   * @throw std::invalid_argument If the parameters have invalid values.
   */
  void fillJointsConfigurationMapFromUrdf(
    const std::vector<hardware_interface::ComponentInfo> & joints);

  /**
   * @brief The Data structure of one joint of the Bravo 7 Manipulator.
   *
   */
  struct ManipulatorJointData
  {
    /// The name of the joint.
    std::string name;
    /// The ID of the joint, according to the map @ref bravo_seven_devices_map_.
    types::DeviceId id;
    /// The map of state interfaces.
    std::unordered_map<std::string, double> state_interfaces;
    /// The map of limit values, the first one the minimum limit and the second one the maximum.
    std::unordered_map<std::string, std::pair<float, float>> limit_values;
  };

  /**
   * @brief Check the parameters of the Joint component.
   *
   * @param valid_joint The joint data, where the parameter values are saved.
   * @return true If the parameters are valid.
   * @return false If the parameters have invalid values.
   */
  bool areJointParametersValid(ManipulatorJointData & valid_joint);

  /**
   * @brief Get the Joint State Interfaces.
   *
   * This method checks if the state interfaces are valid and saves them. Otherwise the
   * state interface will not be exported.
   *
   * @param state_interfaces The vector of state interfaces.
   * @param valid_joint The data of the joint.
   */
  void getJointStateInterfaces(
    const std::vector<hardware_interface::InterfaceInfo> & state_interfaces,
    ManipulatorJointData & valid_joint);

  /**
   * @brief Configure the limits of the joints.
   *
   * This method calls the @ref setJointLimits to configure the positions, current
   * and voltage limits, to all the joints mapped in @ref configurations_map_. If no limit
   * parameter is set for a joint, the limits are not set.
   *
   */
  void configureLimitsOfJoints();

  /**
   * @brief Get the Float Component Parameter object
   *
   * @param joint_data
   * @param float_parameter
   * @param parameter_name
   */
  void getFloatComponentParameter(
    ManipulatorJointData & joint_data, float & float_parameter, std::string parameter_name);

  /**
   * @brief Get the Joint Position Limits Parameters passed through the Urdf.
   *
   * @param joint_name The name of the joint to get the parameters.
   * @param minimum_position The minimum limit to set.
   * @param maximum_position The maximum limit to set.
   */
  void getJointPositionLimitsParameters(
    std::string joint_name, float & minimum_position, float & maximum_position);

  /**
   * @brief The name of the hardware component.
   */
  const std::string hardware_component_name_{"LittlebotHardwareComponent"};

  /**
   * @brief This class uses this pointer to store the concrete LittlebotCommunicationInterface.
   */
  std::shared_ptr<LittlebotCommunicationInterface> manipulator_communication_ptr_;

  /**
   * @brief Unique pointer to an instance of the PluginlibLoaderWrapper class.
   *
   * The ros2_control_toolbox::PluginlibLoaderWrapper loads and unloads a communication plugin,
   * from base class LittlebotCommunicationInterface.
   */
  std::unique_ptr<ros2_control_toolbox::PluginlibLoaderWrapper
    <littlebot_base::LittlebotCommunicationInterface>> plugin_loader_wrapper_ptr_;

  /**
   * @brief The name of the package where the communication plugin is implemented.
   *
   */
  const std::string communication_plugin_package_name_{"littlebot_base"};

  /**
   * @brief The name of the communication plugin.
   *
   */
  std::string communication_plugin_name_{""};

  /**
   * @brief The base class of the communication plugin.
   *
   */
  const std::string communication_plugin_base_class_{"LittlebotCommunicationInterface"};

  /**
   * @brief The driver_base uses this URI to know the communication channel with the manipulator.
   */
  std::string uri_{""};

  /**
   * @brief Default driver_base read timeout.
   */
  std::chrono::milliseconds read_timeout_{0};

  /**
   * @brief Default driver_base write timeout.
   */
  std::chrono::milliseconds write_timeout_{0};

  /**
   * @brief Default read timeout.
   */
  static constexpr std::chrono::milliseconds kDefaultReadTimeout{1000};

  /**
   * @brief Default write timeout.
   */
  static constexpr std::chrono::milliseconds kDefaultWriteTimeout{1000};

  /**
   * @brief The pointer used to access the HardwareDescriptionHelper methods, which gets the
   * hardware component parameters.
   *
   */
  std::unique_ptr<ros2_control_toolbox::HardwareDescriptionHelper> hardware_description_helper_ptr_;

  /**
   * @brief Maps the state interfaces by the joints ID.
   *
   */
  std::unordered_map<std::string, std::vector<types::DeviceId>> state_interfaces_map_;

  /**
   * @brief Maps the configuration parameters by the joints ID.
   *
   */
  std::unordered_map<std::string, std::vector<types::DeviceId>> configurations_map_;

  /**
   * @brief Maps the state interface values of the Force Torque sensor.
   *
   */
  std::unordered_map<std::string, double> force_torque_state_interfaces_map_;

  /**
   * @brief Lists the sensor state interfaces that should be exported.
   *
   */
  std::vector<std::string> sensor_state_interfaces_in_use_;

  /**
   * @brief Maps the configuration parameters with the function related (listed in
   *  @ref LittlebotCommunicationInterface::PacketId).
   *
   */
  std::unordered_map<std::string, types::PacketId> configurations_functions_map_{
    {"maximum_velocity", types::PacketId::SET_VELOCITY_LIMITS},
    {"maximum_current", types::PacketId::SET_CURRENT_LIMITS},
    {"maximum_position", types::PacketId::SET_POSITION_LIMITS},
  };

  /**
   * @brief The data vector of the manipulator's joints.
   *
   */
  std::array<ManipulatorJointData, types::kJointsInManipulator> joints_;

  /**
   * @brief The actual data of the Force Torque sensor.
   *
   */
  types::ForceTorque force_toque_measurements_;

  /**
   * @brief Number of sensors in the Bravo Seven Manipulator.
   *
   */
  static constexpr uint8_t kSensorsInManipulator{1};

  /**
   * @brief The name of the Force Torque sensor.
  */
  std::string sensor_name_{"force_torque"};

  /**
   * @brief Number of state interfaces in the force torque sensor (hardware interface).
   *
   */
  static constexpr uint8_t kSensorStateInterfaces{6};

  /**
   * @brief Number of state interfaces available for each joint.
   *
   */
  static constexpr uint8_t kJointStateInterfacesAvailable{8};

  /**
   * @brief Numbers of parameters by joint.
   *
   */
  static constexpr uint8_t kJointParameters{4};

  /**
   * @brief List of all the state interface names available for one joint.
   *
   */
  std::array<std::string, kJointStateInterfacesAvailable> all_joint_state_interface_names_ =
  {"position", "relative_position", "velocity", "voltage", "current", "internal_humidity",
    "internal_temperature", "internal_pressure"};

  /**
   * @brief List all the configuration parameters available for one joint.
   *
   */
  std::array<std::string, kJointParameters> configuration_parameters_names_ =
  {"maximum_velocity", "maximum_current", "minimum_position", "maximum_position"};

  /**
   * @brief List all the state interface names available for the Force Torque sensor.
   *
   */
  std::array<std::string, kSensorStateInterfaces> all_sensor_state_interface_names_ = {"force_x",
    "force_y", "force_z", "torque_x", "torque_y", "torque_z"};
};

}  // namespace littlebot_base