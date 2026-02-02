// @ Copyright 2025-2026 Nestor Neto
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
#include <vector>

#include "littlebot_base/i_littlebot_driver.hpp"
#include "littlebot_base/i_serial_port.hpp"
#include "littlebot_base/i_rt_buffer.hpp"
#include "littlebot_base/types.hpp"
#include "littlebot_base/wheel.hpp"

namespace littlebot_base
{

class LittlebotDriver : public ILittlebotDriver
{
public:
  /**
   * @brief Construct a new Littlebot Driver object
   *
   * @param serial_port Shared pointer to the serial port interface
   * @param rt_state_buffer Shared pointer to the real-time buffer interface for wheel state data
   * @param rt_command_buffer Shared pointer to the real-time buffer interface for wheel command data
   * @param joint_names Vector of joint names
   * @param port Serial port name
   * @param baudrate Serial port baudrate
   */
  LittlebotDriver(
    std::shared_ptr<ISerialPort> serial_port,
    std::shared_ptr<IRTBuffer<WheelRTData>> rt_state_buffer,
    std::shared_ptr<IRTBuffer<WheelRTData>> rt_command_buffer,
    const std::vector<std::string> & joint_names,
    std::string port,
    int baudrate);

  ~LittlebotDriver() override = default;

  /**
   * @brief Read the current state from the RT buffer
   *
   * \inheritdoc
   * 
   * Override the virtual method from ILittlebotDriver
   */
  void readRTData(WheelRTData & state) const noexcept override;

  /**
   * @brief Write the command to the RT buffer
   *
   * \inheritdoc
   *
   * Override the virtual method from ILittlebotDriver
   */
  void writeRTData(const WheelRTData & command) noexcept override;

  /**
   * @brief Receive data from the hardware and update the RT buffer
   *
   * \inheritdoc
   *
   * Override the virtual method from ILittlebotDriver
   */
  bool requestStatus() noexcept override;

  /**
   * @brief Send command data to the hardware
   *
   * \inheritdoc
   *
   * Override the virtual method from ILittlebotDriver
   */
  bool sendCommand() noexcept override;

  /**
   * @brief Get the last error that occurred
   *
   * \inheritedoc
   *
   * Override the virtual method from ILittlebotDriver
   */
  DriverError getLastError() const noexcept override {return last_error_;}

  /**
   * @brief Get the error counters
   *
   * \inheritedoc
   *
   * Override the virtual method from ILittlebotDriver
   */
  const DriverErrorCounters & getErrorCounters() const noexcept override
  {
    return error_counters_;
  }

private:
  /**
   * @brief Map SerialError to DriverError
   *
   * @param e SerialError to map
   * @return DriverError Mapped DriverError
   */
  static DriverError mapSerialError(SerialError error) noexcept;

  /**
   * @brief Error counters for the driver
   */
  DriverErrorCounters error_counters_;

  /**
   * @brief Last error that occurred
   */
  DriverError last_error_{DriverError::None};

  /**
   * @brief Serial port interface
   */
  std::shared_ptr<ISerialPort> serial_port_;

  /**
   * @brief RT buffer interface for wheel data states
   */
  std::shared_ptr<IRTBuffer<WheelRTData>> rt_state_buffer_;

  /**
   * @brief RT buffer interface for wheel data commands
   */
  std::shared_ptr<IRTBuffer<WheelRTData>> rt_command_buffer_;

  /**
   * @brief Serial port name
   */
  std::string port_;

  /**
   * @brief Serial port baudrate
   */
  int baudrate_{0};

  /**
   * @brief Vector of wheels (Not RT-safe)
   */
  std::vector<littlebot_base::Wheel> wheels_;
};

}  // namespace littlebot_base
