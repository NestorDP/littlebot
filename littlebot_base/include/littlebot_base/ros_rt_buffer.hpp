#pragma once

#include "realtime_tools/realtime_buffer.h"
#include "littlebot_base/i_rt_buffer.hpp"
#include "littlebot_base/wheel_rt_data.hpp"

namespace littlebot_ros2
{

class RosRTBuffer final
: public littlebot_base::IRTBuffer<littlebot_base::WheelRTData>
{
public:
  const littlebot_base::WheelRTData*
  readRT() const noexcept override
  {
    return buffer_.readFromRT();
  }

  void writeNonRT(
    const littlebot_base::WheelRTData& data) override
  {
    buffer_.writeFromNonRT(data);
  }

private:
  realtime_tools::RealtimeBuffer<littlebot_base::WheelRTData> buffer_;
};

}  // namespace littlebot_ros2
