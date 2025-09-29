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

#include <iostream>
#include <string>
#include <vector>

// #include <libserial/serial.hpp>

#include "littlebot_base/i_firmware_comm.hpp"

namespace littlebot_base
{

class FirmwareComm : public IFirmwareComm
{
public:
  /**
   * @brief Constructor for the FirmwareComm class
   */
  explicit FirmwareComm(const std::string & serial_port);

  /**
   * @brief Deconstructor for the FirmwareComm class
   */
  ~FirmwareComm();

  /**
   * @brief Set the command velocities
   */
  void setCommandVelocities([[maybe_unused]] std::vector<float> velocities) override;

  /**
   * @brief Get the status velocities
   */
  std::vector<float> getStatusVelocities() const override;

  /**
   * @brief Get the status positions
   */
  std::vector<float> getStatusPositionsStatus() const override;

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

private:
  /**
   * @brief Receive data from the hardware
   *
   * This function receives the status positions and velocities from the hardware.
   */
  bool receive() override;

  /**
   * @brief Send data to the hardware
   *
   * This function sends the command velocities to the hardware.
   */
  bool send() override;

  /**
   * @brief Encode data to be sent to the hardware
   *
   * This function encodes the command velocities into the protobuf
   * format suitable for transmission to the hardware.
   */
  bool encode() override;

  /**
   * @brief Decode data received from the hardware
   *
   * This function decodes the protobuf data received from the
   * hardware into the status positions and velocities.
   */
  bool decode() override;

  /**
   * @brief Command velocities for the hardware.
   *
   * This vector stores the command velocities that are sent to the hardware.
   */
  std::vector<float> command_velocities_;

  /**
   * @brief Status positions from the hardware.
   *
   * This vector stores the status positions received from the hardware.
   */
  std::vector<float> status_positions_;

  /**
   * @brief Status velocities from the hardware.
   *
   * This vector stores the status velocities received from the hardware.
   */
  std::vector<float> status_velocities_;

  /**
   * @brief Smart pointer to serial port object
   *
   * This object is used to communicate with the hardware. To more information
   * about the serial library used, please visit: https://github.com/NestorDP/libserial
   */
  // std::shared_ptr<serial::Serial> serial_;

  /**
   * @brief Caracter to start the message
   */
  static constexpr char kStartByte{'['};

  /**
   * @brief Caracter to end the message
   */
  static constexpr char kEndByte{']'};
};

}  // namespace littlebot_base
