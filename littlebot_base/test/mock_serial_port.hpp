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
  bool open(std::string port, int baudrate) override
  {
    (void)port;
    (void)baudrate;
    opened_ = true;
    rx_buffer_.clear();
    return true;
  }

  void close() override
  {
    opened_ = false;
  }

  int read(std::string & payload) override
  {
    if (!opened_) {
      return 0;
    }

    // Simulate receiving data from hardware once
    if (!data_injected_) {
      injectWheelFrame();
      data_injected_ = true;
    }

    // Try to extract a frame exactly like SerialPort does
    if (tryExtractFrame(payload)) {
      return static_cast<int>(payload.size());
    }

    return 0;
  }

  int write(const std::string & payload) override
  {
    if (!opened_) {
      return 0;
    }

    last_written_payload_ = payload;
    return static_cast<int>(payload.size());
  }

  // For test assertions
  const std::string & lastWritten() const
  {
    return last_written_payload_;
  }

private:
  // ---- helpers ----

  void injectWheelFrame()
  {
    littlebot::Wheels wheels_msg;

    auto * left = wheels_msg.add_side();
    left->set_command_velocity(1.23f);
    left->set_status_velocity(4.56f);
    left->set_status_position(7.89f);

    auto * right = wheels_msg.add_side();
    right->set_command_velocity(2.34f);
    right->set_status_velocity(5.67f);
    right->set_status_position(8.90f);

    std::string proto;
    wheels_msg.SerializeToString(&proto);

    std::string frame;
    buildFrame(proto, frame);

    // Simulate stream arrival
    rx_buffer_.append(frame);
  }

  void buildFrame(const std::string & payload, std::string & frame)
  {
    frame.clear();
    frame.reserve(payload.size() + 3);

    frame.push_back(kStartByte);        // kStartByte
    frame.append(payload);
    frame.push_back(kEndByte);        // kEndByte
    frame.push_back('\n');
  }

  bool tryExtractFrame(std::string & payload)
  {
    auto start = rx_buffer_.find(kStartByte);
    if (start == std::string::npos) {
      rx_buffer_.clear();
      return false;
    }

    auto end = rx_buffer_.find(kEndByte, start + 1);
    if (end == std::string::npos) {
      return false;
    }

    if (end + 1 >= rx_buffer_.size() || rx_buffer_[end + 1] != '\n') {
      throw std::runtime_error("Malformed mock frame");
    }

    payload.assign(
      rx_buffer_.begin() + start + 1,
      rx_buffer_.begin() + end);

    rx_buffer_.erase(0, end + 2);
    return true;
  }

private:
  bool opened_{false};
  bool data_injected_{false};

  std::string rx_buffer_;
  std::string last_written_payload_;
};
