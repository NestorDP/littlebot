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
// Include generated protobuf messages for constructing test payloads
#include "littlebot_msg.pb.h"  // NOLINT(build/include_subdir)

/**
 * @brief Class for testing SerialPort without actual hardware
 *
 * This class extends SerialPort to allow testing without requiring
 * actual serial hardware connection.
 */
class MockSerialPort : public littlebot_base::ISerialPort
{
public:
  bool open([[maybe_unused]] std::string port, [[maybe_unused]] int baudrate) override
  {
    std::cout << "MockSerialPort opened on port " << port
              << " with baudrate " << baudrate << std::endl;
    return true;
  }

  void close() override {}

  int readPacket(std::shared_ptr<std::string> buffer) override
  {
    // Construct a mock protobuf message
    std::string proto_msg{""};

    // Example protobuf message in hex generated for nanopb
    std::string hex_data{"0A0F0A0000000015DB0FC93F1D81B787400A0F0A00000000153B46713F1DDB0FC93F"};
    for (size_t i = 0; i < hex_data.length(); i += 2) {
        auto byte = std::stoi(hex_data.substr(i, 2), nullptr, 16);
        proto_msg.push_back(byte);
    }

    // Add controller type character at the start of the message
    std::string framed = std::string("S") + proto_msg;

    if (buffer) {
      *buffer = framed;
      return static_cast<int>(buffer->size());
    }
    return 0;
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
