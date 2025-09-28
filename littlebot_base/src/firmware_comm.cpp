//  @ Copyright 2024 Nestor Neto

#include <string>
#include <vector>

#include "littlebot_base/firmware_comm.hpp"

namespace littlebot_base
{
  FirmwareComm::FirmwareComm(const std::string serial_port)
  {
    std::cout << "FirmwareComm constructor" << serial_port << std::endl;

    serial_ = std::make_shared<serial::Serial>();
    serial_->open(serial_port);
  }

  FirmwareComm::~FirmwareComm()
  {
    std::cout << "FirmwareComm deconstructor" << std::endl;
  }

  void FirmwareComm::setCommandVelocities([[maybe_unused]] std::vector<float> velocities)
  {
    std::cout << "FirmwareComm setCommandVelocities" << std::endl;

    for (const auto& vel : velocities) {
      std::cout << vel << " ";
    }
    std::cout << std::endl;
  }

  std::vector<float> FirmwareComm::getStatusVelocities() const
  {
    std::cout << "FirmwareComm getStatusVelocities" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  std::vector<float> FirmwareComm::getStatusPositionsStatus() const
  {
    std::cout << "FirmwareComm getStatusPositionsStatus" << std::endl;
    return std::vector<float>{5.6, 7.8};
  }

  bool FirmwareComm::receive()
  {
    std::cout << "FirmwareComm receive" << std::endl;
    return true;
  }

  bool FirmwareComm::send()
  {
    std::cout << "FirmwareComm send" << std::endl;
    return true;
  }

}  // namespace littlebot_base