// @ Copyright 2025 Nestor Neto
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

#include "littlebot_base/firmware_comm.hpp"

namespace littlebot_base
{
FirmwareComm::FirmwareComm(const std::string & serial_port)
{
  std::cout << "FirmwareComm constructor" << serial_port << std::endl;

  // serial_ = std::make_shared<serial::Serial>();
  // serial_->open(serial_port);
}

FirmwareComm::~FirmwareComm()
{
  std::cout << "FirmwareComm deconstructor" << std::endl;
}

void FirmwareComm::setCommandVelocities([[maybe_unused]] std::vector<float> velocities)
{
  std::cout << "FirmwareComm setCommandVelocities" << std::endl;

  for (const auto & vel : velocities) {
    std::cout << vel << " ";
  }
  std::cout << std::endl;
}

std::vector<float> FirmwareComm::getStatusVelocities() const
{
  std::cout << "FirmwareComm getStatusVelocities" << std::endl;
  return std::vector<float>{1.2, 3.4};
}

std::vector<float> FirmwareComm::getStatusPositionsStatus() const
{
  std::cout << "FirmwareComm getStatusPositionsStatus" << std::endl;
  return std::vector<float>{5.6, 7.8};
}

bool FirmwareComm::receive()
{
  std::cout << "FirmwareComm receive" << std::endl;
  return true;
}

bool FirmwareComm::send()
{
  std::cout << "FirmwareComm send" << std::endl;
  return true;
}
}  // namespace littlebot_base
