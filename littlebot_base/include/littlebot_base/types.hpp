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

  NoData,             // no frame available yet
  DecodeFailure,      // Deserialize error
  EncodeFailure,      // Serialize error
  SizeMismatch,       // wrong number of wheels
  SerialReadError,    // IO error
  SerialWriteError,   // IO error
  NoCommand           // no new command available
};

struct DriverErrorCounters
{
  uint64_t no_data = 0;
  uint64_t decode_failure = 0;
  uint64_t encode_failure = 0;
  uint64_t size_mismatch = 0;
  uint64_t serial_read_error = 0;
  uint64_t serial_write_error = 0;
  uint64_t no_command = 0;

  void reset()
  {
    *this = {};
  }
};

}  // namespace littlebot_base