//  @ Copyright 2024 Nestor Neto

#include <string>
#include <vector>

#include "littlebot_base/littlebot_hardware_comunication.hpp"

namespace littlebot_base
{
  LittlebotHardwareCommunication::LittlebotHardwareCommunication(const std::string serial_port)
  {
    std::cout << "LittlebotHardwareCommunication constructor" << serial_port << std::endl;
    
    serial_ = std::make_shared<serial::Serial>();
    serial_->OpenPort(serial_port);
  }

  LittlebotHardwareCommunication::~LittlebotHardwareCommunication()
  {
    std::cout << "LittlebotHardwareCommunication deconstructor" << std::endl;
  }

  void LittlebotHardwareCommunication::setCommandVelocities([[maybe_unused]] std::vector<float> velocities)
  {
    std::cout << "LittlebotHardwareCommunication setCommandVelocities" << std::endl;
  }

  std::vector<float> LittlebotHardwareCommunication::getStatusVelocities() const
  {
    std::cout << "LittlebotHardwareCommunication getStatusVelocities" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  std::vector<float> LittlebotHardwareCommunication::getStatusPositionsStatus() const
  {
    std::cout << "LittlebotHardwareCommunication getStatusPositionsStatus" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  bool LittlebotHardwareCommunication::receive()
  {
    std::cout << "LittlebotHardwareCommunication receive" << std::endl;
    return true;
  }

  bool LittlebotHardwareCommunication::send()
  {
    std::cout << "LittlebotHardwareCommunication send" << std::endl;
    return true;
  }

}  // namespace littlebot_base