// //  @ Copyright 2024 Nestor Neto

#pragma once

#include <string>
#include <vector>

#include <libserial/serial.hpp>

#include "littlebot_base/hardware_communication.hpp"

namespace littlebot_base
{

class LittlebotHardwareCommunication : public HardwareCommunication
{
public:
  /**
   * @brief Constructor for the LittlebotLittlebotHardwareCommunication class
   */
  LittlebotHardwareCommunication(const std::string serial_port);
  
  /**
   * @brief Deconstructor for the LittlebotHardwareCommunication class
   */
  ~LittlebotHardwareCommunication();

  /**
   * @brief Set the command velocities
   */
  void setCommandVelocities(std::vector<float> velocities) override;

  /**
   * @brief Get the status velocities
   */
  std::vector<float> getStatusVelocities() const override;

  /**
   * @brief Get the status positions
   */
  std::vector<float> getStatusPositionsStatus() const override;

private:
  /**
   * @brief Receive data from the hardware
   */
  bool receive() override;

  /**
   * @brief Send data to the hardware
   */
  bool send() override;

  /**
   * @brief 
   */
  std::vector<float> command_velocities_;

  /**
   * @brief 
   */
  std::vector<float> status_positions_;

  /**
   * @brief 
   */
  std::vector<float> status_velocities_;


};

}  // namespace littlebot_base