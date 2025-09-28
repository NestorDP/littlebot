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
 * @file mock_firmware_comm.cpp
 * @brief Mock implementation of IFirmwareComm for unit testing
 *
 * This file contains a mock class for the IFirmwareComm interface,
 * allowing unit tests to simulate firmware communication without
 * requiring actual hardware.
 * @author Nestor Neto
 * @date 2025
 */

#include <gmock/gmock.h>

#include "littlebot_base/i_firmware_comm.hpp"

class MockFirmwareComm : public IFirmwareComm
{
public:
  MOCK_METHOD(void, setCommandVelocities, (std::vector<float> velocities), (override));
  MOCK_METHOD(std::vector<float>, getStatusVelocities, (), (const, override));
  MOCK_METHOD(std::vector<float>, getStatusPositionsStatus, (), (const, override));
  MOCK_METHOD(bool, start, (), (override));
  MOCK_METHOD(bool, stop, (), (override));
};
