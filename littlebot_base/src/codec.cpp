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

#include <stdexcept>

#include "littlebot_base/codec.hpp"

namespace littlebot_base::codec
{
void encode(
  std::vector<uint8_t> & payload,
  const std::vector<Wheel> & wheels)
{
  littlebot::Wheels send_wheels_data;

  send_wheels_data.mutable_side()->Reserve(wheels.size());

  for (const auto & wheel : wheels) {
    auto * wheel_data = send_wheels_data.add_side();

    wheel_data->set_command_velocity(wheel.getCommandVelocity());
    wheel_data->set_status_velocity(wheel.getStatusVelocity());
    wheel_data->set_status_position(wheel.getStatusPosition());
  }

  const size_t size = send_wheels_data.ByteSizeLong();
  payload.resize(size);

  if (!send_wheels_data.SerializeToArray(payload.data(), static_cast<int>(payload.size()))) {
    throw std::runtime_error("Failed to serialize protobuf message");
  }
}

void decode(
  const std::vector<uint8_t> & payload,
  std::vector<Wheel> & wheels)
{
  if (payload.empty()) {
    throw std::runtime_error("Input payload is empty");
  }

  littlebot::Wheels received_wheels_data;
  if (!received_wheels_data.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
    throw std::runtime_error("Failed to parse protobuf message from input payload");
  }

  const int num_of_side_wheels = received_wheels_data.side_size();
  wheels.resize(num_of_side_wheels);

  for (int i = 0; i < num_of_side_wheels; ++i) {
    const auto & wheel_data = received_wheels_data.side(i);
    wheels[i].setCommandVelocity(wheel_data.command_velocity());
    wheels[i].setStatusVelocity(wheel_data.status_velocity());
    wheels[i].setStatusPosition(wheel_data.status_position());
  }
}

}  // namespace littlebot_base::codec
