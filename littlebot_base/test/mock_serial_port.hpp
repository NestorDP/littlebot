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

/**
 * @file mock_serial_port.hpp
 * @brief Mock class for testing SerialPort without actual hardware
 * @author Nestor Neto
 * @date 2025
 */

#pragma once

#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "littlebot_base/i_serial_port.hpp"

/**
 * @brief Class for testing SerialPort without actual hardware
 *
 * This class extends SerialPort to allow testing without requiring
 * actual serial hardware connection.
 */
class MockSerialPort : public littlebot_base::ISerialPort
{
public:
  bool open(std::string port, int baudrate) override
  {
        // Simulate opening the serial port
        // In a real implementation, you would use libserial to open the port here
        // e.g., serial_.Open(port_path_, baudrate_);

        // For this mock implementation, just print and set is_open_ to true
    std::cout     << "SerialPort open on port: " << port << " with baudrate: "
                  << baudrate << std::endl;
    return true;
  }

  void close() override {}

  int readPacket(std::shared_ptr<std::string> buffer) override
  {
    // ASCII values for the message characters
    std::string message{
      "S0a0f0d0000000015abf4b4401d731d53400a0f0d0000000015000000001d00000000]"
    };

    *buffer = std::move(message);
    return static_cast<int>(buffer->size());
  }

  int writePacket(std::shared_ptr<std::string> buffer) override
  {
    return static_cast<int>(buffer->size());
  }

  int getDataFromPacket(std::shared_ptr<std::string> buffer) override
  {
    return static_cast<int>(buffer->size());
  }
};
