//  @ Copyright 2024 Nestor Neto

#include <string>
#include <vector>

#include "littlebot_base/littlebot_communication.hpp"

namespace littlebot_base
{
  LittlebotCommunication::LittlebotCommunication(const std::string serial_port)
  {
    std::cout << "LittlebotCommunication constructor" << serial_port << std::endl;
    
    serial_ = std::make_shared<serial::Serial>();
    serial_->open(serial_port);
  }

  LittlebotCommunication::~LittlebotCommunication()
  {
    std::cout << "LittlebotCommunication deconstructor" << std::endl;
  }

  void LittlebotCommunication::setCommandVelocities([[maybe_unused]] std::vector<float> velocities)
  {
    std::cout << "LittlebotCommunication setCommandVelocities" << std::endl;
  }

  std::vector<float> LittlebotCommunication::getStatusVelocities() const
  {
    std::cout << "LittlebotCommunication getStatusVelocities" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  std::vector<float> LittlebotCommunication::getStatusPositionsStatus() const
  {
    std::cout << "LittlebotCommunication getStatusPositionsStatus" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  bool LittlebotCommunication::receive()
  {
    std::cout << "LittlebotCommunication receive" << std::endl;
    return true;
  }

  bool LittlebotCommunication::send()
  {
    std::cout << "LittlebotCommunication send" << std::endl;
    return true;
  }

}  // namespace littlebot_base