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
#include <algorithm>

namespace littlebot_base
{

bool SerialPort::open(std::string port, int baudrate)
{
  // TODO(NestorDP): Add error handling
  serial_.open(port);
  serial_.setBaudRate(baudrate);
  is_open_ = true;
  return true;
}

void SerialPort::close()
{
  serial_.close();
  is_open_ = false;
}

int SerialPort::read(std::vector<uint8_t> & payload)
{
  if (is_open_) {
    readStream();
  }

  if (this->tryExtractFrame(payload)) {
    return payload.size();
  }

  return 0;  // no complete frame yet
}

int SerialPort::write(const std::vector<uint8_t> & payload)
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
  auto tmp_buffer = std::make_shared<std::string>();
  size_t n = serial_.read(tmp_buffer, kMaxReadChunk);

  if (n > 0) {
    rx_buffer_.insert(rx_buffer_.end(), tmp_buffer->begin(), tmp_buffer->end());
  }
}

void SerialPort::buildFrame(
  const std::vector<uint8_t> & payload,
  std::string & frame)
{
  frame.clear();
  frame.reserve(payload.size() + 3);

  frame.push_back(kStartByte);
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(kEndByte);
  // frame.push_back('\n');
}

bool SerialPort::tryExtractFrame(std::vector<uint8_t> & payload)
{
  // Safety: prevent unbounded growth
  if (rx_buffer_.size() > kMaxReadChunk) {
    rx_buffer_.erase(rx_buffer_.begin(),
                     rx_buffer_.end() - kMaxReadChunk);
  }

  auto start = std::find(rx_buffer_.begin(), rx_buffer_.end(), kStartByte);
  if (start == rx_buffer_.end()) {
    // No frame start: discard garbage
    rx_buffer_.clear();
    return false;
  }

  auto end = std::find(start + 1, rx_buffer_.end(), kEndByte);
  if (end == rx_buffer_.end()) {
    // Start found, but frame not complete yet
    return false;
  }

  payload.assign(start + 1, end);

  // Drop everything up to and including the end byte
  rx_buffer_.erase(rx_buffer_.begin(), end + 1);

  // Remove trailing delimiters
  while (!rx_buffer_.empty() &&
    (rx_buffer_.front() == '\n' || rx_buffer_.front() == '\r'))
  {
    rx_buffer_.erase(rx_buffer_.begin());
  }

  return true;
}
}  // namespace littlebot_base
