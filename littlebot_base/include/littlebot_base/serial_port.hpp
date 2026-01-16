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
#include <memory>
#include <string>
#include <vector>

#include "littlebot_base/i_serial_port.hpp"

namespace littlebot_base
{

class SerialPort : public ISerialPort
{
public:
  /**
   * @brief Open the serial port (uses stored parameters)
   */
  bool open(std::string port, int baudrate) override;

  /**
   * @brief Close the serial port
   */
  void close() override;

  /**
   * @brief Read packet data from the serial port
   */
  int read(std::string & payload) override;

  /**
   * @brief Write packet data to the serial port
   */
  int write(const std::string & payload) override;

  #ifdef UNIT_TEST
    void injectRxData(const std::string & data)
    {
      rx_buffer_.append(data);
    }
  #endif

private:
  /**
   * @brief Read data stream from the serial port
   */
  void readStream() override;

  /**
   * @brief Get data from the received packet
   *
   * @param buffer Shared pointer to string buffer to store received data
   */
  bool tryExtractFrame(std::string & payload) override;

  /**
   * @brief Build packet to be sent through serial port
   *
   * @param buffer Shared pointer to string buffer to store data to be sent
   */
  void buildFrame(
    const std::string & payload,
    std::string & frame) override;

  /**
   * @brief Serial port device path (e.g., "/dev/ttyUSB0")
   */
  std::string port_path_{"/dev/rfcomm0"};

  /**
   * @brief Communication baudrate (e.g., 9600, 115200)
   */
  int baudrate_{115200};
};

}  // namespace littlebot_base
