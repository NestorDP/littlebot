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
 
}

FirmwareComm::~FirmwareComm()
{
  std::cout << "FirmwareComm deconstructor" << std::endl;
}

void FirmwareComm::setCommandVelocities(std::map<std::string, float> velocities)
{
  std::cout << "FirmwareComm setCommandVelocities" << std::endl;

  // Store the velocities in the member variable
  command_velocities_ = velocities;

  // for (const auto & vel : velocities) {
  //   std::cout << vel << " ";
  // }
  // std::cout << std::endl;
}

std::map<std::string, float> FirmwareComm::getStatusVelocities() const
{
  std::cout << "FirmwareComm getStatusVelocities" << std::endl;
  return {{"left_wheel", 1.2}, {"right_wheel", 3.4}};
}

std::map<std::string, float> FirmwareComm::getStatusPositions() const
{
  std::cout << "FirmwareComm getStatusPositions" << std::endl;
  return {{"left_wheel", 5.6}, {"right_wheel", 7.8}};
}

bool FirmwareComm::start()
{
  std::cout << "FirmwareComm start" << std::endl;
  return true;
}

bool FirmwareComm::stop()
{
  std::cout << "FirmwareComm stop" << std::endl;
  return true;
}

int FirmwareComm::receiveDataPacket()
{  
  // // Check if serial port is available
  // if (!serial_port_) {
  //   std::cerr << "Serial port not initialized" << std::endl;
  //   return -1;
  // }
  
  // // Create temporary buffer to receive data
  // std::vector<uint8_t> temp_buffer;
  
  // // Read data from serial port
  // int bytes_read = serial_port_->read(temp_buffer);
  
  // if (bytes_read > 0) {
  //   // Append received data to input buffer
  //   input_buffer_.insert(input_buffer_.end(), temp_buffer.begin(), temp_buffer.end());
    
  //   std::cout << "Read " << bytes_read << " bytes, input buffer size: " 
  //             << input_buffer_.size() << std::endl;
  // } else if (bytes_read == 0) {
  //   // No data available
  //   std::cout << "No data available" << std::endl;
  // } else {
  //   // Error occurred
  //   std::cerr << "Error reading from serial port" << std::endl;
  // }
  
  // return bytes_read;
}

bool FirmwareComm::sendDataPacket()
{
  std::cout << "FirmwareComm sendDataPacket" << std::endl;
  return true;
}

bool FirmwareComm::encode()
{
  std::cout << "FirmwareComm encode" << std::endl;
  return true;
}

bool FirmwareComm::decode()
{
  std::cout << "FirmwareComm decode" << std::endl;
  return true;
}

std::vector<uint8_t> FirmwareComm::getInputBuffer() const
{
  return input_buffer_;
}

void FirmwareComm::clearInputBuffer()
{
  input_buffer_.clear();
  std::cout << "Input buffer cleared" << std::endl;
}

}  // namespace littlebot_base
