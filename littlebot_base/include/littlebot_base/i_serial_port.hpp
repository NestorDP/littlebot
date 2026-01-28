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
#include <memory>
#include <string>
#include <vector>

/**
 * To more information about the serial library used,
 * please visit: https://github.com/NestorDP/cppserial
 */
#include "libserial/serial.hpp"


namespace littlebot_base
{

class ISerialPort
{
public:
  /**
   * @brief Deconstructor for the ISerialPort class
   *
   */
  virtual ~ISerialPort() = default;

  /**
   * @brief Open the serial port (uses default or stored parameters)
   */
  virtual bool open(std::string port, int baudrate) = 0;

  /**
   * @brief Close the serial port
   */
  virtual void close() = 0;

  /**
   * @brief Read Packet from the serial port
   */
  virtual int read(std::vector<uint8_t> & payload) = 0;

  /**
   * @brief Write Packet to the serial port
   */
  virtual int write(const std::vector<uint8_t> & payload) = 0;

  /**
   * @brief Check if the serial port is open
   *
   * @return true if the serial port is open
   * @return false if the serial port is closed
   */
  virtual bool isOpen() const noexcept
  {
    return is_open_;
  }

protected:
  /**
   * @brief Constructor for the ISerialPort class
   */
  ISerialPort() = default;

  /**
   * @brief Prevent copy and assignment
   */
  ISerialPort(const ISerialPort &) = delete;
  ISerialPort & operator=(const ISerialPort &) = delete;

  /**
   * @brief Read data stream from the serial port
   */
  virtual void readStream() = 0;

  /**
   * @brief Get data from the received packet
   *
   * @param buffer Shared pointer to string buffer to store received data
   */
  virtual bool tryExtractFrame(std::vector<uint8_t> & payload) = 0;

  /**
   * @brief Build packet to be sent through serial port
   *
   * @param buffer Shared pointer to string buffer to store data to be sent
   */
  virtual void buildFrame(
    const std::vector<uint8_t> & payload,
    std::string & frame) = 0;

  /**
   * @brief Buffer to store received data
   */
  std::vector<uint8_t> rx_buffer_;

  /**
   * @brief Serial object from libserial
   */
  libserial::Serial serial_;

  /**
   * @brief Caracter to start the message
   */
  static constexpr char kStartByte{'['};

  /**
   * @brief Caracter to end the message
   */
  static constexpr char kEndByte{']'};

  /**
   * @brief Flag to indicate if the serial port is open
   */
  bool is_open_{false};
};

}  // namespace littlebot_base
