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

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/**
 * To more information about the serial library used,
 * please visit: https://github.com/NestorDP/cppserial
 */
#include "libserial/serial.hpp"


namespace littlebot_base
{

enum class SerialError
{
  None,
  PortUnavailable,
  InsufficientPermissions,
  ConfigBaudrateFailed,
  ReadFailed,
  WriteFailed,
  AlreadyOpen,
  NotOpen,
  NotClosed,
  Unknown
};

class ISerialPort
{
public:
  /**
   * @brief Maximum size of a frame that can be sent/received
   */
  static constexpr size_t kMaxFrameSize = 256;

  /**
   * @brief Deconstructor for the ISerialPort class
   *
   */
  virtual ~ISerialPort() = default;

  /**
   * @brief Open the serial port
   *
   * @param port Serial port device path (e.g., "/dev/ttyUSB0")
   * @param baudrate Baud rate for the serial communication
   * @return SerialError indicating success or type of failure
   */
  virtual SerialError open(std::string port, int baudrate) noexcept = 0;

  /**
   * @brief Close the serial port
   *
   * @return SerialError indicating success or type of failure
   */
  virtual SerialError close() noexcept = 0;

  /**
   * @brief Read Packet from the serial port
   *
   * @param payload Vector to store the read packet data.
   *        Implementations must ensure the frame size never exceeds kMaxFrameSize.
   * @return Number of bytes read
   */
  virtual int read(std::vector<uint8_t> & payload) noexcept = 0;

  /**
   * @brief Write Packet to the serial port
   *
   * @param payload Vector containing the packet data to write
   *        payload.size() must be <= kMaxFrameSize.
   * @return Number of bytes written
   */
  virtual int write(const std::vector<uint8_t> & payload) noexcept = 0;

  /**
   * @brief Check if the serial port is open
   *
   * @return true if the serial port is open
   * @return false if the serial port is closed
   */
  virtual bool isOpen() const noexcept
  {
    return is_open_;
  }

protected:
  /**
   * @brief Constructor for the ISerialPort class
   */
  ISerialPort() = default;

  /**
   * @brief Prevent copy and assignment
   */
  ISerialPort(const ISerialPort &) = delete;
  ISerialPort & operator=(const ISerialPort &) = delete;

  /**
   * @brief Read data stream from the serial port
   */
  virtual void readStream() noexcept = 0;

  /**
   * @brief Get data from the received packet
   *
   * @param buffer Shared pointer to string buffer to store received data
   * @return true if a complete frame was extracted
   * @return false if no complete frame was available
   */
  virtual bool tryExtractFrame(std::vector<uint8_t> & payload) noexcept = 0;

  /**
   * @brief Build packet to be sent through serial port
   *
   * @param payload Vector containing the packet data to send
   * @param frame String to store the built packet
   */
  virtual void buildFrame(
    const std::vector<uint8_t> & payload,
    std::string & frame) noexcept = 0;

  /**
   * @brief Buffer to store received data
   */
  std::vector<uint8_t> rx_buffer_;

  /**
   * @brief Serial object from libserial
   */
  libserial::Serial serial_;

  /**
   * @brief Character to start the message
   */
  static constexpr char kStartByte{'['};

  /**
   * @brief Character to end the message
   */
  static constexpr char kEndByte{']'};

  /**
   * @brief Flag to indicate if the serial port is open
   */
  bool is_open_{false};
};

}  // namespace littlebot_base
