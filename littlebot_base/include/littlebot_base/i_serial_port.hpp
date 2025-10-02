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
#include <string>
#include <vector>

// #include "libserial/SerialPort.hpp"
// To more information
//    * about the serial library used, please visit: https://github.com/NestorDP/libserial

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
  virtual bool open() = 0;

  /**
   * @brief Close the serial port
   */
  virtual void close() = 0;

  /**
   * @brief Read data from the serial port
   */
  virtual int readPacket(std::vector<uint8_t> & buffer) = 0;

  /**
   * @brief Write data to the serial port
   */
  virtual int writePacket(const std::vector<uint8_t> & buffer) = 0;

  /** 
   * @brief Prevent copy and assignment
   */
  ISerialPort(const ISerialPort&) = delete;
  ISerialPort& operator=(const ISerialPort&) = delete;

protected:
  /**
   * @brief Constructor for the ISerialPort class
   */
  ISerialPort() = default;
};

}  // namespace littlebot_base
