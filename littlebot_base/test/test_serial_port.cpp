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


#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "littlebot_base/serial_port.hpp"

//
TEST(TestSerialPort, GetDataFromPacket)
{
    littlebot_base::SerialPort serial_port;

    std::string packet{"[TEST_DATA]"};
    std::shared_ptr<std::string> input_buffer = std::make_shared<std::string>(packet);

    packet.erase(0, 1);
    packet.pop_back();

    serial_port.getDataFromPacket(input_buffer);
    EXPECT_STREQ(input_buffer->c_str(), packet.c_str());
}
