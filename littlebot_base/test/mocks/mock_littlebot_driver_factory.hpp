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

#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <vector>
#include "littlebot_base/i_littlebot_driver_factory.hpp"

namespace littlebot_base
{

class MockLittlebotDriverFactory : public ILittlebotDriverFactory
{
public:
  MOCK_METHOD(std::shared_ptr<ILittlebotDriver>, create,
    (const std::string &, int, const std::vector<std::string> &),
    (override));
};

}  // namespace littlebot_base
