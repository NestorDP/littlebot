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


#include "littlebot_base/i_serial_port.hpp"
#include "littlebot_base/i_rt_buffer.hpp"
#include "littlebot_base/types.hpp"
#include "littlebot_base/wheel.hpp"

namespace littlebot_base
{

class LittlebotDriver
{
public:
  static constexpr char kCommandChar = 'C';
  static constexpr char kStatusChar = 'S';

  /**
   * @brief Construct a new Littlebot Driver object
   * 
   * @param serial_port Shared pointer to the serial port interface
   * @param rt_buffer Shared pointer to the real-time buffer interface for wheel data
   */
  LittlebotDriver(
    std::shared_ptr<ISerialPort> serial_port,
    std::shared_ptr<IRTBuffer<WheelRTData>> rt_state_buffer,
    std::shared_ptr<IRTBuffer<WheelRTData>> rt_command_buffer);

  /**
   * @brief Read the current state from the RT buffer
   * 
   * @param state Reference to WheelRTData structure to store the read data
   * 
   * @note This method is RT-safe (control loop)
   */
  void readRTData(WheelRTData & state) const noexcept;

  /**
   * @brief Write the command to the RT buffer
   * 
   * @param command Reference to WheelRTData structure containing the command data
   * 
   * @note This method is RT-safe (control loop)
   */
  void writeRTData(const WheelRTData & command) noexcept;

  /**
   * @brief Receive data from the hardware and update the RT buffer
   * 
   * @return true if data was received successfully
   * @return false if an error occurred
   * 
   * @note This method is NOT RT-safe (executor / IO thread)
   */
  bool requestStatus();

  /**
   * @brief Send command data to the hardware
   * 
   * @return true if data was sent successfully
   * @return false if an error occurred
   * 
   * @note This method is NOT RT-safe (executor / IO thread)
   */
  bool sendCommand();

  /**
   * @brief Get the last error that occurred
   * 
   * @return DriverError The last error code
   */
  DriverError getLastError() const noexcept { return last_error_; }

  /**
   * @brief Get the error counters
   * 
   * @return const DriverErrorCounters& Reference to the error counters structure
   */
  const DriverErrorCounters & getErrorCounters() const noexcept
  {
    return error_counters_;
  }
  
private:
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
   * @brief Vector of wheels (Not RT-safe)
   */
  std::vector<littlebot_base::Wheel> wheels_;
};

}  // namespace littlebot_base
