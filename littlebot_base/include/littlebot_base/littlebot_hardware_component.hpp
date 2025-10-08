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
#include "littlebot_base/i_serial_port.hpp"
#include "littlebot_base/serial_port.hpp"

namespace littlebot_base
{

class LittlebotHardwareComponent : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Constructor for the LittlebotHardwareComponent class
   */
  LittlebotHardwareComponent() = default;

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
   * @brief Setup the Littlebot driver with the given serial port parameters
   */
  void setupDriver(
    std::shared_ptr<littlebot_base::ISerialPort> serial_port,
    const std::string & port, int baudrate);

private:
  /**
   * @brief The name of the hardware component.
   */
  const std::string hardware_component_name_{"LittlebotHardwareComponent"};

  /**
   * @brief Shared pointer to the Littlebot driver
   */
  std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;

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
};

}  // namespace littlebot_base
