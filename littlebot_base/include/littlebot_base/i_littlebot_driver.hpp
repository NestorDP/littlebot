// @ Copyright 2026 Nestor Neto
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

#include "littlebot_base/i_serial_port.hpp"
#include "littlebot_base/i_rt_buffer.hpp"
#include "littlebot_base/types.hpp"
#include "littlebot_base/wheel.hpp"

namespace littlebot_base
{

class ILittlebotDriver
{
public:
  /**
   * @brief Command control characters
   */
  static constexpr char kCommandChar{'C'};

  /**
   * @brief Status control characters
   */
  static constexpr char kStatusChar{'S'};

  /**
   * @brief Construct a new Littlebot Driver object
   *
   * @param serial_port Shared pointer to the serial port interface
   * @param rt_buffer Shared pointer to the real-time buffer interface for wheel data
   */
  ILittlebotDriver(
    std::shared_ptr<ISerialPort> serial_port,
    std::shared_ptr<IRTBuffer<WheelRTData>> rt_state_buffer,
    std::shared_ptr<IRTBuffer<WheelRTData>> rt_command_buffer,
    const std::vector<std::string> & joint_names)
  {
    (void)serial_port;
    (void)rt_state_buffer;
    (void)rt_command_buffer;
    (void)joint_names;
  }

  /**
   * @brief Prevent copy and assignment
   */
  ILittlebotDriver(const ILittlebotDriver &) = delete;
  ILittlebotDriver & operator=(const ILittlebotDriver &) = delete;

  /**
   * @brief Deconstructor for the ILittlebotDriver class
   */
  virtual ~ILittlebotDriver() = default;

  /**
   * @brief Read the current state from the RT buffer
   *
   * @param state Reference to WheelRTData structure to store the read data
   *
   * @note This method is RT-safe (control loop)
   */
  virtual void readRTData(WheelRTData & state) const noexcept = 0;

  /**
   * @brief Write the command to the RT buffer
   *
   * @param command Reference to WheelRTData structure containing the command data
   *
   * @note This method is RT-safe (control loop)
   */
  virtual void writeRTData(const WheelRTData & command) noexcept = 0;

  /**
   * @brief Receive data from the hardware and update the RT buffer
   *
   * @return true if data was received successfully
   * @return false if an error occurred
   *
   * @note This method is NOT RT-safe (executor / IO thread)
   */
  virtual bool requestStatus() noexcept = 0;

  /**
   * @brief Send command data to the hardware
   *
   * @return true if data was sent successfully
   * @return false if an error occurred
   *
   * @note This method is NOT RT-safe (executor / IO thread)
   */
  virtual bool sendCommand() noexcept = 0;

  /**
   * @brief Get the last error that occurred
   *
   * @return DriverError The last error code
   */
  virtual DriverError getLastError() const noexcept = 0;

  /**
   * @brief Get the error counters
   *
   * @return const DriverErrorCounters& Reference to the error counters structure
   */
  virtual const DriverErrorCounters & getErrorCounters() const noexcept = 0;
};

}  // namespace littlebot_base
