#include "littlebot_base/littlebot_driver_factory.hpp"
#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/serial_port.hpp"
#include "littlebot_base/ros_rt_buffer.hpp"

namespace littlebot_base {

std::shared_ptr<ILittlebotDriver>
createLittlebotDriver(
  const std::string & port,
  int baudrate,
  const std::vector<std::string> joint_names)
{
  auto serial = std::make_shared<SerialPort>();
  auto state_buffer = std::make_shared<RosRTBuffer>();
  auto cmd_buffer = std::make_shared<RosRTBuffer>();

  if (!serial->open(port, baudrate)) {
    return nullptr;
  }

  return std::make_shared<LittlebotDriver>(
    serial, state_buffer, cmd_buffer, joint_names);
}

}  // namespace littlebot_base