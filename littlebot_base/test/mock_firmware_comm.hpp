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
 * @file mock_firmware_comm.hpp
 * @brief Mock class for testing FirmwareComm without actual hardware
 * @author Nestor Neto
 * @date 2025
 */

#pragma once

#include <vector>
#include <string>
#include <memory>

#include "littlebot_base/firmware_comm.hpp"

/**
 * @brief Class for testing FirmwareComm without actual hardware
 *
 * This class extends FirmwareComm to allow testing without requiring
 * actual serial hardware connection.
 */
class MockableFirmwareComm : public littlebot_base::FirmwareComm
{
public:
  /**
   * @brief Constructor for mock firmware communication
   * @param serial_port Serial port path (defaults to /dev/ttyUSB0 for testing)
   */
  explicit MockableFirmwareComm(const std::string & serial_port = "/dev/ttyUSB0");

  /**
   * @brief Destructor
   */
  ~MockableFirmwareComm() = default;

  // Test helper methods to access internal state
  void setTestCommandVelocities(const std::vector<float> & velocities);
  void setTestStatusVelocities(const std::vector<float> & velocities);
  void setTestStatusPositions(const std::vector<float> & positions);
  std::vector<float> getTestCommandVelocities() const;

  // Override methods for testing (if needed)
  // bool mockStart();
  // bool mockStop();

private:
  std::vector<float> test_command_velocities_;
  std::vector<float> test_status_velocities_;
  std::vector<float> test_status_positions_;
};