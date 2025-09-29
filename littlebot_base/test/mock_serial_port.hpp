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
  bool open(const std::string&, int) override { return true; }

  void close() override {}

  int readPacket(std::vector<uint8_t>& buffer) override {
    if (readBuffer.empty()) return 0;
    auto resp = readBuffer.front(); readBuffer.pop();
    buffer = resp;
    return buffer.size();
  }

  int writePacket(const std::vector<uint8_t> & buffer) override {
    writtenBuffer.insert(writtenBuffer.end(), buffer.begin(), buffer.end());
      return buffer.size();
  }

  /**
   * @brief Add test data from hex string to read buffer
   */
  void addTestData(const std::vector<uint8_t>& data) {
    readBuffer.push(data);
  }

  /**
   * @brief Add complete protobuf message with frame format: [<control_char><hex_data>]
   */
  void addCompleteMessage(const std::string& message) {
    std::vector<uint8_t> data(message.begin(), message.end());
    readBuffer.push(data);
  }

  /* Buffers for written and read data */
  std::vector<uint8_t> writtenBuffer;
  std::queue<std::vector<uint8_t>> readBuffer;
};