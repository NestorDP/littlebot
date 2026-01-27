#pragma once

#include <memory>
#include <string>
#include <vector>

namespace littlebot_base {

class ILittlebotDriver;

std::shared_ptr<ILittlebotDriver>
createLittlebotDriver(
  const std::string & port,
  int baudrate,
  const std::vector<std::string> & joint_names);

}  // namespace littlebot_base