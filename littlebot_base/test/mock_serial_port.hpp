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
    return true;
  }

  void close() override {}

  int readPacket(std::shared_ptr<std::string> buffer) override
  {
    littlebot::Wheels wheels_msg;

    littlebot::WheelData * wheel_left = wheels_msg.add_side();
    wheel_left->set_command_velocity(1.23f);
    wheel_left->set_status_velocity(4.56f);
    wheel_left->set_status_position(7.89f);

    littlebot::WheelData * wheel_right = wheels_msg.add_side();
    wheel_right->set_command_velocity(2.34f);
    wheel_right->set_status_velocity(5.67f);
    wheel_right->set_status_position(8.90f);

    std::string proto;
    if (!wheels_msg.SerializeToString(&proto)) {
      proto.clear();
    }

    std::string framed = std::string("S") + proto;

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
