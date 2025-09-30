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
    SerialPort::SerialPort(const std::string& port, int baudrate)
    : port_path_(port), baudrate_(baudrate), is_open_(false)
    {
        std::cout << "SerialPort constructor: " << port << " @ " << baudrate << " baud" << std::endl;
        // Automatically open the port during construction
        is_open_ = open();
    }

    bool SerialPort::open()
    {
        if (port_path_.empty()) {
            std::cerr << "Error: No port specified. Use constructor with port parameter." << std::endl;
            return false;
        }
        
        std::cout << "SerialPort open on port: " << port_path_ << " with baudrate: " << baudrate_ << std::endl;
        is_open_ = true;
        return true;
    }

    void SerialPort::close()
    {
        if (is_open_) {
            std::cout << "SerialPort close: " << port_path_ << std::endl;
            is_open_ = false;
        } else {
            std::cout << "SerialPort already closed" << std::endl;
        }
    }

    int SerialPort::readPacket([[maybe_unused]] std::vector<uint8_t>& buffer)
    {
        // // Check if we have minimum frame size: [<controller>]
        // if (input_buffer_.size() < 3) {
        //     std::cerr << "Received frame too short: " << input_buffer_.size() << " bytes" << std::endl;
        //     return 0;
        // }

        // // Check for start byte '['
        // if (input_buffer_[0] != static_cast<uint8_t>(kStartByte)) {
        //     std::cerr << "Invalid start byte: expected '[', got " << static_cast<char>(input_buffer_[0]) << std::endl;
        //     return 0;
        // }
        
        // // Check for end byte ']'
        // if (input_buffer_.back() != static_cast<uint8_t>(kEndByte)) {
        //     std::cerr << "Invalid end byte: expected ']', got " << static_cast<char>(input_buffer_.back()) << std::endl;
        //     return 0;
        // }

        if (!is_open_) {
            std::cerr << "Error: Cannot read from closed serial port" << std::endl;
            return -1;
        }

        std::cout << "SerialPort readPacket from " << port_path_ << std::endl;
        return 0;
    }

    int SerialPort::writePacket([[maybe_unused]] const std::vector<uint8_t> & buffer)
    {
        if (!is_open_) {
            std::cerr << "Error: Cannot write to closed serial port" << std::endl;
            return -1;
        }
        std::cout << "SerialPort writePacket with size: " << buffer.size() << std::endl;
        return buffer.size();
    }

    const std::string& SerialPort::getPortPath() const
    {
        return port_path_;
    }

    int SerialPort::getBaudrate() const
    {
        return baudrate_;
    }

    bool SerialPort::isOpen() const
    {
        return is_open_;
    }
}  // namespace littlebot_base
