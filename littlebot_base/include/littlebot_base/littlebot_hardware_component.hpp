// @ Copyright 2025 Nestor Neto
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
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/serial_port.hpp"
#include "littlebot_base/ros_rt_buffer.hpp"
#include "littlebot_base/i_littlebot_driver_factory.hpp"
#include "littlebot_base/i_rt_buffer.hpp"
#include "littlebot_base/types.hpp"

namespace littlebot_base
{

class LittlebotHardwareComponent : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Default constructor for the LittlebotHardwareComponent class
   */
  LittlebotHardwareComponent() = default;

  /**
   * @brief Constructor for the LittlebotHardwareComponent class
   *
   * @param driver Shared pointer to the Littlebot driver
   * @note This constructor is mainly used for testing purposes
   */
  explicit LittlebotHardwareComponent(
    std::shared_ptr<ILittlebotDriver> driver)
  : littlebot_driver_(std::move(driver)) {}

  /**
   * @brief Deconstructor for the LittlebotHardwareComponent class
   */
  ~LittlebotHardwareComponent() = default;

  /**
   * @brief Initialize the hardware component with the given parameters
   */
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  override;

  /**
   * @brief Configure the hardware communication
   */
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state)
  override;

  /**
   * @brief
   */
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  override;

  /**
   * @brief
   */
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state)
  override;

  /**
   * @brief Export the state interfaces
   */
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces()
  override;

  /**
   * @brief Export the command interfaces
   */
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces()
  override;

  /**
   * @brief Read the state of the hardware component
   */
  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period)
  override;

  /**
   * @brief Write the command to the hardware component
   */
  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period)
  override;

  /**
   * @brief Set the Littlebot driver factory
   *
   * @param factory Shared pointer to the Littlebot driver factory
   * @note This method is mainly used for testing purposes
   */
  void setDriverFactory(std::shared_ptr<ILittlebotDriverFactory> factory)
  {
    driver_factory_ = std::move(factory);
  }

private:
  /**
   * @brief The name of the hardware component.
   */
  const std::string hardware_component_name_{"LittlebotHardwareComponent"};

  /**
   * @brief Shared pointer to the Littlebot driver
   */
  std::shared_ptr<littlebot_base::ILittlebotDriver> littlebot_driver_;

  /**
   * @brief Shared pointer to the Littlebot driver factory
   */
  std::shared_ptr<ILittlebotDriverFactory> driver_factory_;

  /**
   * @brief Shared pointer to the RT buffer for wheel states
   */
  std::shared_ptr<IRTBuffer<WheelRTData>> rt_state_buffer_;

  /**
   * @brief Shared pointer to the RT buffer for wheel commands
   */
  std::shared_ptr<IRTBuffer<WheelRTData>> rt_command_buffer_;

  /**
   * @brief Non-RT IO timer
   */
  rclcpp::TimerBase::SharedPtr io_timer_;

  /**
   * @brief Serial port device name
   */
  std::string serial_port_name_;

  /**
   * @brief Serial port baudrate
   */
  int serial_baudrate_{115200};

  /**
   * @brief command interface.
   */
  std::vector<double> hw_commands_velocities_;

  /**
   * @brief position state interface.
   */
  std::vector<double> hw_status_positions_;

  /**
   * @brief velocity state interface.
   */
  std::vector<double> hw_status_velocities_;

  /**
   * @brief constant for number of command interfaces.
   */
  static constexpr int kNumCommandInterface_{1};

  /**
   * @brief constant for number of state interfaces.
   */
  static constexpr int kNumStateInterface_{2};
};

}  // namespace littlebot_base
