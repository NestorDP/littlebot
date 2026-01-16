// @ Copyright 2026 Nestor Neto
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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace littlebot_base
{

class Wheel
{
public:
  Wheel() = default;

  explicit Wheel(std::string joint_name)
  : joint_name_(std::move(joint_name)) {}

  ~Wheel() = default;

  /**
   * @brief Set the command velocity for the wheel
   * 
   * @param velocity Desired command velocity
   */
  void setCommandVelocity(double velocity) {command_vel_ = velocity;}

  /**
   * @brief Set the joint name for the wheel
   * 
   * @param joint_name Name of the joint
   */
  void setJointName(const std::string & joint_name) {joint_name_ = joint_name;}

  /**
   * @brief Set the status velocity for the wheel
   * 
   * @param velocity Measured status velocity
   */
  void setStatusVelocity(double velocity) {status_vel_ = velocity;}

  /**
   * @brief Set the status position for the wheel
   * 
   * @param position Measured status position
   */
  void setStatusPosition(double position) {status_pos_ = position;}

  /**
   * @brief Get the command velocity for the wheel
   */
  double getCommandVelocity() const {return command_vel_;}

  /**
   * @brief Get the status position for the wheel
   */
  double getStatusPosition() const {return status_pos_;}

  /**
   * @brief Get the status velocity for the wheel
   */
  double getStatusVelocity() const {return status_vel_;}

  /**
   * @brief Get the joint name for the wheel
   */
  std::string getJointName() const {return joint_name_;}

private:
  /**
   * @brief Status velocity of the wheel
   */
  double status_vel_{0.0};

  /**
   * @brief Status position of the wheel
   */
  double status_pos_{0.0};

  /**
   * @brief Command velocity of the wheel
   */
  double command_vel_{0.0};

  /**
   * @brief Joint name of the wheel
   */
  std::string joint_name_{"wheel"};
};

}  // namespace littlebot_base
