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

/**
 * @file mock_rt_buffer.hpp
 * @brief Mock class for testing RTBuffer without actual real-time context
 * @author Nestor Neto
 * @date 2026
 */

#pragma once

#include <gmock/gmock.h>
#include "littlebot_base/i_rt_buffer.hpp"

template<typename T>
class MockRTBuffer : public littlebot_base::IRTBuffer<T>
{
public:
  MOCK_METHOD(const T *, readRT, (), (const, noexcept, override));
  MOCK_METHOD(void, writeNonRT, (const T &), (override));
};
