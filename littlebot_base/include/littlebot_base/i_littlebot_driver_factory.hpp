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

#include <expected>
#include <memory>
#include <string>
#include <vector>

#include "littlebot_base/types.hpp"

namespace littlebot_base
{

class ILittlebotDriver;

class ILittlebotDriverFactory
{
public:
  virtual ~ILittlebotDriverFactory() = default;

  virtual std::expected<std::shared_ptr<ILittlebotDriver>, DriverError> create(
    const std::string & port,
    int baudrate,
    const std::vector<std::string> & joint_names) = 0;
};

}  // namespace littlebot_base
