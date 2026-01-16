// @ Copyright 2025-2026 Nestor Neto
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

#include <gmock/gmock.h>
#include <string>
#include "littlebot_base/i_serial_port.hpp"

class MockSerialPort : public littlebot_base::ISerialPort
{
public:
  MockSerialPort() {}
  ~MockSerialPort() override = default;
  
  MOCK_METHOD(bool, open, (std::string port, int baudrate), (override));
  MOCK_METHOD(void, close, (), (override));
  MOCK_METHOD(int, write, (const std::string &), (override));
  MOCK_METHOD(int, read, (std::string &), (override));
  
  // Provide default implementations for pure virtual methods
  void readStream() override {}
  bool tryExtractFrame([[maybe_unused]] std::string & payload) override {return true;}
  void buildFrame(const std::string & payload, std::string & frame) override {frame = payload;}
};
