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

#include <array>
#include <cstddef>
#include <cstdint>

namespace littlebot_base
{

constexpr size_t kNumWheels = 2;

struct WheelRTData
{
  std::array<float, kNumWheels> command_velocity{};
  std::array<float, kNumWheels> status_velocity{};
  std::array<float, kNumWheels> status_position{};
};

enum class DriverError
{
  None = 0,

  NoData,              // 1: No frame available yet
  DecodeFailure,       // 2: Deserialize error
  EncodeFailure,       // 3: Serialize error
  SizeMismatch,        // 4: Wrong number of wheels
  SerialReadError,     // 5: IO error
  SerialWriteError,    // 6: IO error
  InvalidControlChar,  // 7: Unexpected control character
  NoCommand            // 8: No new command available
};

struct DriverErrorCounters
{
  uint64_t no_data = 0;
  uint64_t decode_failure = 0;
  uint64_t encode_failure = 0;
  uint64_t size_mismatch = 0;
  uint64_t serial_read_error = 0;
  uint64_t serial_write_error = 0;
  uint64_t invalid_control_char = 0;
  uint64_t no_command = 0;

  void reset()
  {
    *this = {};
  }
};

}  // namespace littlebot_base
