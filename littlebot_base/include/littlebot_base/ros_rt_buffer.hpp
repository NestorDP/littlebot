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

#include "realtime_tools/realtime_buffer.h"
#include "littlebot_base/i_rt_buffer.hpp"
#include "littlebot_base/types.hpp"

namespace littlebot_ros2
{

class RosRTBuffer final
  : public littlebot_base::IRTBuffer<littlebot_base::WheelRTData>
{
public:
  const littlebot_base::WheelRTData *
  readRT() const noexcept override
  {
    return buffer_.readFromRT();
  }

  void writeNonRT(
    const littlebot_base::WheelRTData & data) override
  {
    buffer_.writeFromNonRT(data);
  }

private:
  realtime_tools::RealtimeBuffer<littlebot_base::WheelRTData> buffer_;
};

}  // namespace littlebot_ros2
