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
#include "littlebot_base/serial_port.hpp"
#include <stdexcept>

namespace littlebot_base
{
FirmwareComm::FirmwareComm(std::shared_ptr<littlebot_base::ISerialPort> serial_port)
: serial_port_(serial_port)
{
  // Validate that the serial port was provided
  if (!serial_port_) {
    std::cerr << "Error: Null serial port provided to FirmwareComm constructor" << std::endl;
    throw std::invalid_argument("Serial port cannot be null");
  }
  
  std::cout << "FirmwareComm initialized with provided serial port" << std::endl;
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

uint8_t FirmwareComm::receiveData()
{    
  int bytes_read = serial_port_->readPacket(input_buffer_);
  if (bytes_read < 0) {
    std::cerr << "Failed to read data from serial port" << std::endl;
    return 0;
  }
  // Extract controller character (first byte after start frame)
  uint8_t controller_character = input_buffer_[1];

  // Remove frame bytes and controller character, keeping only the data
  // Original: [C<data>] -> Result: <data>
  std::vector<uint8_t> clean_data(input_buffer_.begin() + 2, input_buffer_.end() - 1);
  
  // Replace input_buffer_ with clean data (without frame and controller)
  input_buffer_ = clean_data;
  
  return controller_character;
}


bool FirmwareComm::sendData(uint8_t type)
{
  std::cout << "FirmwareComm sendData with type: " << static_cast<char>(type) << std::endl;

  // Create the complete message with framing: [<type><data>]
  std::vector<uint8_t> complete_message;

  complete_message.push_back(static_cast<uint8_t>(kStartByte));
  complete_message.push_back(type);
  complete_message.insert(complete_message.end(), output_buffer_.begin(), output_buffer_.end());
  complete_message.push_back(static_cast<uint8_t>(kEndByte));

  int bytes_written = serial_port_->writePacket(complete_message);
  if (bytes_written < 0) {
    std::cerr << "Failed to write data to serial port" << std::endl;
    return false;
  }
  
  std::cout << "Successfully sent " << bytes_written << " bytes with frame format [" 
            << static_cast<char>(type) << "<data>]" << std::endl;
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
}

}  // namespace littlebot_base
