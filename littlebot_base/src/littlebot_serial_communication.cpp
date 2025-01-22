//  @ Copyright 2024 Nestor Neto

#include <string>
#include <vector>

#include "littlebot_base/littlebot_serial_communication.hpp"

namespace littlebot_base
{
  LittlebotSerialCommunication::LittlebotSerialCommunication(const std::string serial_port)
  {
    std::cout << "LittlebotSerialCommunication constructor" << serial_port << std::endl;
    
    serial_ = std::make_shared<serial::Serial>();
    serial_->open(serial_port);
  }

  LittlebotSerialCommunication::~LittlebotSerialCommunication()
  {
    std::cout << "LittlebotSerialCommunication deconstructor" << std::endl;
  }

  void LittlebotSerialCommunication::setCommandVelocities([[maybe_unused]] std::vector<float> velocities)
  {
    std::cout << "LittlebotSerialCommunication setCommandVelocities" << std::endl;
  }

  std::vector<float> LittlebotSerialCommunication::getStatusVelocities() const
  {
    std::cout << "LittlebotSerialCommunication getStatusVelocities" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  std::vector<float> LittlebotSerialCommunication::getStatusPositionsStatus() const
  {
    std::cout << "LittlebotSerialCommunication getStatusPositionsStatus" << std::endl;
    return std::vector<float>{1.2, 3.4};
  }

  bool LittlebotSerialCommunication::receive()
  {
    std::cout << "LittlebotSerialCommunication receive" << std::endl;
    return true;
  }

  bool LittlebotSerialCommunication::send()
  {
    std::cout << "LittlebotSerialCommunication send" << std::endl;
    return true;
  }

}  // namespace littlebot_base