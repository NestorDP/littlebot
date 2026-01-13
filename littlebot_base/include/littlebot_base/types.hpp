#pragma once

#include <array>
#include <cstddef>

namespace littlebot_base
{

constexpr size_t kNumWheels = 2;

struct WheelRTData
{
  std::array<float, kNumWheels> command_velocity{};
  std::array<float, kNumWheels> status_velocity{};
  std::array<float, kNumWheels> status_position{};
};

}  // namespace littlebot_base