// @ Copyright 2025-2026 Nestor Neto
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

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/codec.hpp"

namespace littlebot_base
{

LittlebotDriver::LittlebotDriver(
  std::shared_ptr<ISerialPort> serial_port,
  std::shared_ptr<IRTBuffer<WheelRTData>> rt_state_buffer,
  std::shared_ptr<IRTBuffer<WheelRTData>> rt_command_buffer,
  const std::vector<std::string> & joint_names,
  std::string port,
  int baudrate)
: serial_port_(std::move(serial_port)),
  rt_state_buffer_(std::move(rt_state_buffer)),
  rt_command_buffer_(std::move(rt_command_buffer)),
  port_(std::move(port)),
  baudrate_(baudrate)
{
  wheels_.reserve(joint_names.size());

  for (const auto & name : joint_names) {
    wheels_.emplace_back(name);
  }

  serial_port_->open(port_, baudrate_);
}

void LittlebotDriver::readRTData(WheelRTData & state) const noexcept
{
  const WheelRTData * data = rt_state_buffer_->readRT();
  if (data) {
    state = *data;
  }
}

void LittlebotDriver::writeRTData(const WheelRTData & command) noexcept
{
  rt_command_buffer_->writeNonRT(command);
}

bool LittlebotDriver::requestStatus() noexcept
{
  // Add control char to request status in payload
  std::vector<uint8_t> payload{kStatusChar};

  if (serial_port_->write(payload) <= 0) {
    return false;
  }

  int num_chars_read = 0;
  try {
    num_chars_read = serial_port_->read(payload);
  } catch (...) {
    ++error_counters_.serial_read_error;
    last_error_ = DriverError::SerialReadError;
    return false;
  }

  if (num_chars_read <= 0) {
    ++error_counters_.no_data;
    last_error_ = DriverError::NoData;
    return false;
  }

  // Check and strip control character
  const char control = payload.front();
  if (control != kStatusChar && control != kCommandChar) {
    ++error_counters_.invalid_control_char;
    last_error_ = DriverError::InvalidControlChar;
    return false;
  }

  // Remove first character (S or C)
  payload.erase(payload.begin());

  try {
    codec::decode(payload, wheels_);
  } catch (...) {
    ++error_counters_.decode_failure;
    last_error_ = DriverError::DecodeFailure;
    return false;
  }

  if (wheels_.size() != kNumWheels) {
    ++error_counters_.size_mismatch;
    last_error_ = DriverError::SizeMismatch;
    return false;
  }

  WheelRTData rt_data{};
  for (size_t i = 0; i < kNumWheels; ++i) {
    rt_data.status_position[i] = wheels_[i].getStatusPosition();
    rt_data.status_velocity[i] = wheels_[i].getStatusVelocity();
  }

  rt_state_buffer_->writeNonRT(rt_data);
  last_error_ = DriverError::None;
  return true;
}

bool LittlebotDriver::sendCommand() noexcept
{
  const auto * rt_data = rt_command_buffer_->readRT();
  if (!rt_data) {
    ++error_counters_.no_command;
    last_error_ = DriverError::NoCommand;
    return false;
  }

  // Update wheels_ from RT command
  for (size_t i = 0; i < kNumWheels; ++i) {
    wheels_[i].setCommandVelocity(rt_data->command_velocity[i]);
  }

  // Encode protobuf
  std::vector<uint8_t> payload;
  codec::encode(payload, wheels_);

  // Prepend control char
  payload.insert(payload.begin(), kCommandChar);

  return serial_port_->write(payload) > 0;
}

static DriverError mapSerialError(SerialError error) noexcept
{
  switch (error) {
    case SerialError::None:        return DriverError::None;
    // case SerialError::ReadFailed:  return DriverError::SerialReadError;
    // case SerialError::WriteFailed: return DriverError::SerialWriteError;
    // case SerialError::OpenFailed:  return DriverError::SerialOpenError;
    default:                       return DriverError::None;
  }
}
}  // namespace littlebot_base
