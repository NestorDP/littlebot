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
 * @brief Mock implementation for testing FirmwareComm without actual hardware
 * @author Nestor Neto
 * @date 2025
 */

#include "mock_firmware_comm.hpp"

MockableFirmwareComm::MockableFirmwareComm(const std::string & serial_port)
: littlebot_base::FirmwareComm(serial_port)
{
  // Initialize test data
  test_command_velocities_ = {0.0f, 0.0f};
  test_status_velocities_ = {0.0f, 0.0f};
  test_status_positions_ = {0.0f, 0.0f};
}

void MockableFirmwareComm::setTestCommandVelocities(const std::vector<float> & velocities)
{
  test_command_velocities_ = velocities;
}

void MockableFirmwareComm::setTestStatusVelocities(const std::vector<float> & velocities)
{
  test_status_velocities_ = velocities;
}

void MockableFirmwareComm::setTestStatusPositions(const std::vector<float> & positions)
{
  test_status_positions_ = positions;
}

std::vector<float> MockableFirmwareComm::getTestCommandVelocities() const
{
  return test_command_velocities_;
}
