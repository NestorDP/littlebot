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
#include "littlebot_base/i_littlebot_driver.hpp"

namespace littlebot_base
{

class MockLittlebotDriver : public ILittlebotDriver
{
public:
  MOCK_METHOD(void, readRTData, (WheelRTData &), (const, noexcept, override));
  MOCK_METHOD(void, writeRTData, (const WheelRTData &), (noexcept, override));
  MOCK_METHOD(bool, requestStatus, (), (noexcept, override));
  MOCK_METHOD(bool, sendCommand, (), (noexcept, override));
  MOCK_METHOD(DriverError, getLastError, (), (const, noexcept, override));
  MOCK_METHOD(const DriverErrorCounters &, getErrorCounters,
    (), (const, noexcept, override));
};

}  // namespace littlebot_base
