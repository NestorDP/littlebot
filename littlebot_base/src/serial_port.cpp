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
    bool SerialPort::open(const std::string& port, int baudrate)
    {
    std::cout << "SerialPort open on port: " << port << " with baudrate: " << baudrate << std::endl;
    return true;
    }

    void SerialPort::close()
    {
    std::cout << "SerialPort close" << std::endl;
    }

    int SerialPort::readPacket(std::vector<uint8_t>& buffer)
    {
    std::cout << "SerialPort readPacket" << std::endl;
    return 0;
    }

    int SerialPort::writePacket(const std::vector<uint8_t> & buffer)
    {
    std::cout << "SerialPort writePacket with size: " << buffer.size() << std::endl;
    return buffer.size();
    }
}
