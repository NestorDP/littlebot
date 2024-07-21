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
  // ~LittlebotHardwareComponent() override;

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
   * @brief The name of the hardware component.
   */
  const std::string hardware_component_name_{"LittlebotHardwareComponent"};

  /**
   * @brief command interface.
   */
  std::vector<double> hw_commands_;

  /**
   * @brief position state interface.
   */
  std::vector<double> hw_positions_;

  /**
   * @brief velocity state interface.
   */
  std::vector<double> hw_velocities_;

  /**
   * @brief 
   */
  static constexpr int kNumCommandInterface_{1};

  /**
   * @brief 
   */
  static constexpr int kNumStateInterface_{2};











  /**
   * @brief 
   */
  //std::shared_ptr<LittlebotCommunicationInterface> littlebot_communication_ptr_;

  /**
   * @brief The name of the package where the communication plugin is implemented.
   *
   */
  //const std::string communication_plugin_package_name_{"littlebot_base"};

  /**
   * @brief The name of the communication plugin.
   *
   */
  //std::string communication_plugin_name_{""};

  /**
   * @brief The base class of the communication plugin.
   *
   */
  //const std::string communication_plugin_base_class_{"LittlebotCommunicationInterface"};

};

}  // namespace littlebot_base