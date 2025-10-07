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
LittlebotDriver::LittlebotDriver(
  std::shared_ptr<littlebot_base::ISerialPort> serial_port,
  std::string port,
  int baudrate)
: serial_port_(serial_port)
{
  // Validate that the serial port was provided
  if (!serial_port_) {
    throw std::invalid_argument("Serial port cannot be null");
  }
  // Open the serial port with default parameters
  if (!serial_port_->open(port, baudrate)) {
    throw std::runtime_error("Failed to open serial port");
  }
  // Initialize buffers
  input_buffer_ = std::make_shared<std::string>();
  output_buffer_ = std::make_shared<std::string>();
}

LittlebotDriver::~LittlebotDriver()
{
  serial_port_->close();
  serial_port_.reset();
}

void LittlebotDriver::setCommandVelocities(std::map<std::string, float> velocities)
{
  command_velocities_ = velocities;
}

std::map<std::string, float> LittlebotDriver::getStatusVelocities() const
{
  return status_velocities_;
}

std::map<std::string, float> LittlebotDriver::getStatusPositions() const
{
  return status_positions_;
}

char LittlebotDriver::receiveData()
{
  int bytes_read = serial_port_->readPacket(input_buffer_);
  if (bytes_read < 0) {
    throw std::invalid_argument("Zero bytes read from serial port");
  }

  // Extract controller character (first byte after start frame)
  char controller_char{input_buffer_->front()};
  input_buffer_->erase(0, 1);
  this->decode();

  return controller_char;
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
  try {
    wheels_data_.Clear();

    for (const auto & wheel_name : wheel_names_) {
      littlebot::WheelData * wheel_data = wheels_data_.add_side();

      // Set command velocity (from command_velocities_ map)
      auto cmd_vel_it = command_velocities_.find(wheel_name);
      if (cmd_vel_it != command_velocities_.end()) {
        wheel_data->set_command_velocity(cmd_vel_it->second);
      } else {
        wheel_data->set_command_velocity(0.0f);  // Default value
      }

      // Set status velocity (from status_velocities_ map)
      auto status_vel_it = status_velocities_.find(wheel_name);
      if (status_vel_it != status_velocities_.end()) {
        wheel_data->set_status_velocity(status_vel_it->second);
      } else {
        wheel_data->set_status_velocity(0.0f);  // Default value
      }

      // Set status position (from status_positions_ map)
      auto status_pos_it = status_positions_.find(wheel_name);
      if (status_pos_it != status_positions_.end()) {
        wheel_data->set_status_position(status_pos_it->second);
      } else {
        wheel_data->set_status_position(0.0f);  // Default value
      }
    }

    if (!wheels_data_.SerializeToString(output_buffer_.get())) {
      std::cerr << "Error: Failed to serialize protobuf message" << std::endl;
      return false;
    }

    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error during encoding: " << e.what() << std::endl;
    return false;
  }
}

bool LittlebotDriver::decode()
{
  try {
    // Check if input buffer has data
    if (!input_buffer_ || input_buffer_->empty()) {
      std::cerr << "Error: Input buffer is empty or null" << std::endl;
      return false;
    }

    // Parse the protobuf message from the input buffer
    littlebot::Wheels received_wheels_data;
    if (!received_wheels_data.ParseFromString(*input_buffer_)) {
      std::cerr << "Error: Failed to parse protobuf message from input buffer" << std::endl;
      return false;
    }

    int wheel_count = received_wheels_data.side_size();

    for (int i = 0; i < wheel_count && i < static_cast<int>(wheel_names_.size()); ++i) {
      const littlebot::WheelData & wheel_data = received_wheels_data.side(i);
      const std::string & wheel_name = wheel_names_[i];

      // Extract and store the values from protobuf to maps
      if (wheel_data.has_command_velocity()) {
        command_velocities_[wheel_name] = wheel_data.command_velocity();
      }

      if (wheel_data.has_status_velocity()) {
        status_velocities_[wheel_name] = wheel_data.status_velocity();
      }

      if (wheel_data.has_status_position()) {
        status_positions_[wheel_name] = wheel_data.status_position();
      }
    }

    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error during decoding: " << e.what() << std::endl;
    return false;
  }
}

std::shared_ptr<std::string> LittlebotDriver::getInputBuffer() const
{
  return input_buffer_;
}

std::shared_ptr<std::string> LittlebotDriver::getOutputBuffer() const
{
  return output_buffer_;
}

std::vector<std::string> LittlebotDriver::getWheelNames() const
{
  return wheel_names_;
}

}  // namespace littlebot_base
