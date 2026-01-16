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

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "littlebot_msg.pb.h"  // NOLINT(build/include_subdir)
#include "littlebot_base/wheel.hpp"

namespace littlebot_base::codec
{
  /**
   * @brief Encode wheel command data into a protobuf payload
   *
   * This function serializes the command velocities of the wheels
   * into a protobuf message suitable for transmission to the hardware.
   *
   * @param payload Output payload that will contain the serialized protobuf message
   * @param wheels Vector of wheels providing command data
   *
   * @throws std::runtime_error if serialization fails
   */
  void encode(
    std::string & payload,
    const std::vector<Wheel> & wheels);

  /**
   * @brief Decode wheel state data from a protobuf payload
   *
   * This function deserializes a protobuf message received from the
   * hardware and updates the wheel status fields. Existing wheel
   * metadata (e.g., joint names) is preserved.
   *
   * @param payload Input payload containing the serialized protobuf message
   * @param wheels Vector of wheels to be updated with decoded state data
   *
   * @throws std::runtime_error if parsing fails or data size mismatches
   */
  void decode(
    const std::string & payload,
    std::vector<Wheel> & wheels);
}  // namespace littlebot_base::codec
