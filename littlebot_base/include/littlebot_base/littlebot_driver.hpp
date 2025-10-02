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

#include <cstdint>
#include <map>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "littlebot_msg.pb.h"  // NOLINT(build/include_subdir)
#include "littlebot_base/i_serial_port.hpp"

namespace littlebot_base
{

class LittlebotDriver
{
public:
  /**
   * @brief Constructor for the LittlebotDriver class
   *
   * @param serial_port Shared pointer to the serial port implementation
   */
  explicit LittlebotDriver(std::shared_ptr<littlebot_base::ISerialPort> serial_port);

  /**
   * @brief Deconstructor for the LittlebotDriver class
   */
  ~LittlebotDriver();

  /**
   * @brief Set the command velocities
   */
  void setCommandVelocities(std::map<std::string, float> velocities);

  /**
   * @brief Get the status velocities
   */
  std::map<std::string, float> getStatusVelocities() const;

  /**
   * @brief Get the status positions
   */
  std::map<std::string, float> getStatusPositions() const;

  /**
   * @brief Start threads
   *
   * This function starts the threads that read and write data to the hardware.
   */
  bool start();

  /**
   * @brief Stop thread that read data from serial port
   *
   * This function stops the threads that read and write data to the hardware.
   */
  bool stop();

  /**
   * @brief Receive data from the hardware
   *
   * @return The controller character indicating the type of data received.
   */
  bool receiveData();

  /**
   * @brief Send data to the hardware
   *
   * @param type Character indicating the type of data being sent.
   *             'S' to status and 'C' to command.
   * @return True if data was sent successfully, false otherwise
   */
  bool sendData(char type);

  /**
   * @brief Encode data to be sent to the hardware
   *
   * This function encodes the command velocities into the protobuf
   * format suitable for transmission to the hardware.
   */
  bool encode();

  /**
   * @brief Decode data received from the hardware
   *
   * This function decodes the protobuf data received from the
   * hardware into the status positions and velocities.
   */
  bool decode();

  /**
   * @brief Get the current input buffer contents (for testing)
   *
   * @return Copy of the input buffer
   */
  std::shared_ptr<std::string> getInputBuffer() const;

  /**
   * @brief Clear the input buffer (for testing)
   */
  void clearInputBuffer();

private:
  /**
   * @brief Data structure for wheels
   *
   * This structure holds the wheel data including positions and velocities.
   */
  littlebot::Wheels wheels_data_;

  /**
   * @brief Command velocities for the hardware.
   *
   * This map stores the command velocities that are sent to the hardware.
   */
  std::map<std::string, float> command_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  /**
   * @brief Status positions from the hardware.
   *
   * This map stores the status positions received from the hardware.
   */
  std::map<std::string, float> status_positions_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  /**
   * @brief Status velocities from the hardware.
   *
   * This map stores the status velocities received from the hardware.
   */
  std::map<std::string, float> status_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  /**
   * @brief Smart pointer to serial_port object
   *
   * This object is used to communicate with the hardware.
   */
  std::shared_ptr<littlebot_base::ISerialPort> serial_port_;

  /**
   * @brief Input buffer for assembling incoming messages
   *
   * This buffer accumulates incoming bytes until a complete message is formed.
   */
  std::shared_ptr<std::string> input_buffer_;

  /**
   * @brief Output buffer for sending messages
   *
   * This buffer holds the encoded message ready to be sent to the hardware.
   */
  std::shared_ptr<std::string> output_buffer_;
};

}  // namespace littlebot_base
