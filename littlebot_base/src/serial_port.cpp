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

#include "littlebot_base/serial_port.hpp"

namespace littlebot_base
{

bool SerialPort::open(std::string port, int baudrate)
{
  try {
    serial_.open(port);
    serial_.setBaudRate(baudrate);
  } catch(const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  std::cout << "SerialPort open on port: " << port << " with baudrate: "
            << baudrate << std::endl;
  return true;
}

void SerialPort::close()
{
  serial_.close();
  std::cout << "SerialPort already closed" << std::endl;
}

int SerialPort::readPacket(std::shared_ptr<std::string> buffer)
{
  // Check if we have minimum frame size: [<controller>]
  int num_characters = serial_.getAvailableData();
  if (num_characters < 3) {
    std::cerr << "Received frame too short: " << num_characters
              << " bytes" << std::endl;
    return -1;
  }

  serial_.read(buffer, num_characters);
  int result = this->getDataFromPacket(buffer);
  if (result < 0) {
    return -1;
  }
  return result;
}

int SerialPort::writePacket(std::shared_ptr<std::string> buffer)
{
  buffer->insert(buffer->begin(), kStartByte);
  buffer->push_back(kEndByte);
  buffer->push_back('\n');

  serial_.write(buffer);
  return buffer->size();
}

int SerialPort::getDataFromPacket(std::shared_ptr<std::string> buffer)
{
  if (buffer->front() == kStartByte) {
    buffer->erase(0, 1);
  } else {
    std::cerr << "Invalid start byte: expected " << kStartByte << ", got "
              << static_cast<char>(buffer->front()) << std::endl;
    return -1;
  }

  if (buffer->back() == kEndByte) {
    buffer->pop_back();
  } else {
    std::cerr << "Invalid end byte: expected " << kEndByte << ", got "
              << static_cast<char>(buffer->back()) << std::endl;
    return -1;
  }
  return buffer->size();
}
}  // namespace littlebot_base
