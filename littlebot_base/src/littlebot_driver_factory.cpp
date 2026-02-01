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

#include "littlebot_base/littlebot_driver_factory.hpp"
#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/serial_port.hpp"
#include "littlebot_base/ros_rt_buffer.hpp"

namespace littlebot_base
{

std::shared_ptr<ILittlebotDriver>
LittlebotDriverFactory::create(
  const std::string & port,
  int baudrate,
  const std::vector<std::string> & joint_names)
{
  auto serial = std::make_shared<SerialPort>();
  auto state_buffer = std::make_shared<RosRTBuffer>();
  auto cmd_buffer = std::make_shared<RosRTBuffer>();

  return std::make_shared<LittlebotDriver>(
    serial, state_buffer, cmd_buffer, joint_names, port, baudrate);
}

}  // namespace littlebot_base
