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
   * @brief 
   */
  void setCommandVelocity(double velocity){ command_vel_ = velocity;}

  /**
   * @brief 
   */
  void setJointName(const std::string & joint_name){ joint_name_ = joint_name; }

  /**
   * @brief
   */
  void setStatusVelocity(double velocity){ status_vel_ = velocity; }

  /**
   * @brief
   */
  void setStatusPosition(double position){ status_pos_ = position; }

  /**
   * @brief
   */
  double getCommandVelocity() const { return command_vel_; }

  /**
   * @brief
   */
  double getStatusPosition() const { return status_pos_; }

  /**
   * @brief
   */
  double getStatusVelocity() const { return status_vel_; }

  /**
   * @brief
   */
  std::string getJointName() const { return joint_name_; }

private:
  /**
   * @brief 
   */
  double status_vel_{0.0};

  /**
   * @brief 
   */
  double status_pos_{0.0};

  /**
   * @brief 
   */
  double command_vel_{0.0};

  /**
   * @brief 
   */
  std::string joint_name_{"wheel"};
};

}  // namespace littlebot_base
