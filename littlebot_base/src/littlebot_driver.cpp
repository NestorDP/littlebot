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

#include "littlebot_base/littlebot_driver.hpp"

#include <stdexcept>

#include "littlebot_base/serial_port.hpp"

namespace littlebot_base
{
LittlebotDriver::LittlebotDriver(std::shared_ptr<littlebot_base::ISerialPort> serial_port)
: serial_port_(serial_port)
{
  // Validate that the serial port was provided
  if (!serial_port_) {
    std::cerr << "Error: Null serial port provided to LittlebotDriver constructor" << std::endl;
    throw std::invalid_argument("Serial port cannot be null");
  }

  std::cout << "LittlebotDriver initialized with provided serial port" << std::endl;
}

LittlebotDriver::~LittlebotDriver()
{
  std::cout << "LittlebotDriver deconstructor" << std::endl;
}

void LittlebotDriver::setCommandVelocities(std::map<std::string, float> velocities)
{
  std::cout << "LittlebotDriver setCommandVelocities" << std::endl;

  // Store the velocities in the member variable
  command_velocities_ = velocities;

  // for (const auto & vel : velocities) {
  //   std::cout << vel << " ";
  // }
  // std::cout << std::endl;
}

std::map<std::string, float> LittlebotDriver::getStatusVelocities() const
{
  std::cout << "LittlebotDriver getStatusVelocities" << std::endl;
  return {{"left_wheel", 1.2}, {"right_wheel", 3.4}};
}

std::map<std::string, float> LittlebotDriver::getStatusPositions() const
{
  std::cout << "LittlebotDriver getStatusPositions" << std::endl;
  return {{"left_wheel", 5.6}, {"right_wheel", 7.8}};
}

bool LittlebotDriver::start()
{
  std::cout << "LittlebotDriver start" << std::endl;
  return true;
}

bool LittlebotDriver::stop()
{
  std::cout << "LittlebotDriver stop" << std::endl;
  return true;
}

bool LittlebotDriver::receiveData()
{
  int bytes_read = serial_port_->readPacket(input_buffer_);
  if (bytes_read < 0) {
    std::cerr << "Failed to read data from serial port" << std::endl;
    return 0;
  }

  // Extract controller character (first byte after start frame)
  input_buffer_->erase(0, 1);
  this->decode();

  return true;
}


bool LittlebotDriver::sendData(char type)
{
  // Create the complete message with framing: [<type><data>]
  output_buffer_->push_back(type);
  this->encode();
  int bytes_written = serial_port_->writePacket(output_buffer_);
  if (bytes_written < 0) {
    std::cerr << "Failed to write data to serial port" << std::endl;
    return false;
  }
  return true;
}

bool LittlebotDriver::encode()
{
  std::cout << "LittlebotDriver encode" << std::endl;
  return true;
}

bool LittlebotDriver::decode()
{
  std::cout << "LittlebotDriver decode" << std::endl;
  return true;
}

std::shared_ptr<std::string> LittlebotDriver::getInputBuffer() const
{
  return input_buffer_;
}

void LittlebotDriver::clearInputBuffer()
{
  input_buffer_->clear();
}

}  // namespace littlebot_base
