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

#include "littlebot_base/i_serial_port.hpp"

namespace littlebot_base
{

class SerialPort : public ISerialPort
{
public:
  /**
   * @brief Constructor for the SerialPort class
   */
  SerialPort() = default;

  /**
   * @brief Open the serial port
   */
  virtual bool open(const std::string& port, int baudrate) override;

  /**
   * @brief Close the serial port
   */
  virtual void close() override;

  /**
   * @brief Read data from the serial port
   */
  virtual int readPacket(std::vector<uint8_t>& buffer) override;

  /**
   * @brief Write data to the serial port
   */
  virtual int writePacket(const std::vector<uint8_t> & buffer) override;

};

}  // namespace littlebot_base
