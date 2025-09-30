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
   * @brief Constructor that opens the serial port with specified parameters
   * @param port The serial port device path (e.g., "/dev/ttyUSB0")
   * @param baudrate The communication speed (e.g., 9600, 115200)
   */
  SerialPort(const std::string& port, int baudrate);

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;


  /**
   * @brief Open the serial port (uses stored parameters)
   */
  bool open() override;

  /**
   * @brief Close the serial port
   */
  void close() override;

  /**
   * @brief Read data from the serial port
   */
  int readPacket(std::vector<uint8_t>& buffer) override;

  /**
   * @brief Write data to the serial port
   */
  int writePacket(const std::vector<uint8_t> & buffer) override;

  /**
   * @brief Get the current port path
   * @return The serial port device path
   */
  const std::string& getPortPath() const;

  /**
   * @brief Get the current baudrate
   * @return The communication baudrate
   */
  int getBaudrate() const;

  /**
   * @brief Check if the port is open
   * @return True if port is open, false otherwise
   */
  bool isOpen() const;

private:
  /**
   * @brief Serial port device path (e.g., "/dev/ttyUSB0")
   */
  std::string port_path_;

  /**
   * @brief Communication baudrate (e.g., 9600, 115200)
   */
  int baudrate_{115200};

  /**
   * @brief Flag indicating if the port is currently open
   */
  bool is_open_{false};

};

}  // namespace littlebot_base
