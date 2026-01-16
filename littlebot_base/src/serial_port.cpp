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
  serial_.setBaudRate(baudrate);
  serial_.open(port);
  is_open_ = true;
  return true;
}

void SerialPort::close()
{
  serial_.close();
  is_open_ = false;
}

int SerialPort::read(std::string & payload)
{
  if (is_open_) {
    readStream();
  }

  if (this->tryExtractFrame(payload)) {
    return payload.size();
  }

  return 0;  // no complete frame yet
}

int SerialPort::write(const std::string & payload)
{
  auto frame = std::make_shared<std::string>();
  buildFrame(payload, *frame);

  if (is_open_) {
    serial_.write(frame);
  }

  return static_cast<int>(frame->size());
}

void SerialPort::readStream()
{
  constexpr size_t kMaxReadChunk = 256;

  auto tmp = std::make_shared<std::string>();
  size_t n = serial_.read(tmp, kMaxReadChunk);

  if (n > 0) {
    rx_buffer_.append(*tmp);
  }
}

void SerialPort::buildFrame(
  const std::string & payload,
  std::string & frame)
{
  frame.clear();
  frame.reserve(payload.size() + 3);

  frame.push_back(kStartByte);
  frame.append(payload);
  frame.push_back(kEndByte);
  frame.push_back('\n');
}

bool SerialPort::tryExtractFrame(std::string & payload)
{
  while (!rx_buffer_.empty() && rx_buffer_.front() == '\r') {
    rx_buffer_.erase(0, 1);
  }

  auto start = rx_buffer_.find(kStartByte);
  if (start == std::string::npos) {
    rx_buffer_.clear();
    return false;
  }

  if (start > 0) {
    rx_buffer_.erase(0, start);
  }

  auto end = rx_buffer_.find(kEndByte, 1);
  if (end == std::string::npos) {
    return false;
  }

  payload.assign(
    rx_buffer_.begin() + 1,
    rx_buffer_.begin() + end);

  size_t consume_len = end + 1;

  if (consume_len < rx_buffer_.size() && rx_buffer_[consume_len] == '\n') {
    ++consume_len;
  }

  if (consume_len < rx_buffer_.size() && rx_buffer_[consume_len] == '\r') {
    ++consume_len;
  }

  rx_buffer_.erase(0, consume_len);

  return true;
}
}  // namespace littlebot_base
