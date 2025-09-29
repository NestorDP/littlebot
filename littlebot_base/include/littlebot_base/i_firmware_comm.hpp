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

#pragma once

#include <string>
#include <vector>

namespace littlebot_base
{

class IFirmwareComm
{
public:
  /**
   * @brief Deconstructor for the IFirmwareComm class
   *
   */
  virtual ~IFirmwareComm() = default;

  /**
   * @brief Set the command velocities
   */
  virtual void setCommandVelocities(std::vector<float> velocities) = 0;

  /**
   * @brief Get the status velocities
   */
  virtual std::vector<float> getStatusVelocities() const = 0;

  /**
   * @brief Get the status positions
   */
  virtual std::vector<float> getStatusPositionsStatus() const = 0;

protected:
  /**
   * @brief Constructor for the IFirmwareComm class
   */
  IFirmwareComm() = default;

private:
  /**
   * @brief Receive data from the hardware
   */
  virtual bool receive() = 0;

  /**
   * @brief Send data to the hardware
   */
  virtual bool send() = 0;

  /**
   * @brief Encode data to be sent to the hardware
   */
  virtual bool encode() = 0;

  /**
   * @brief Decode data received from the hardware
   */
  virtual bool decode() = 0;

  /**
   * @brief
   */
  std::vector<float> command_velocities_;

  /**
   * @brief
   */
  std::vector<float> status_positions_;

  /**
   * @brief
   */
  std::vector<float> status_velocities_;
};

}  // namespace littlebot_base
