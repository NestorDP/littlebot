// //  @ Copyright 2024 Nestor Neto

#pragma once

#include <string>
#include <vector>

namespace littlebot_base
{

class IFirmwareComm
{
 public:
  /**
   * @brief Deconstructor for the IFirmwareComm class
   *
   */
  virtual ~IFirmwareComm() = default;

  /**
   * @brief Set the command velocities
   */
  virtual void setCommandVelocities(std::vector<float> velocities) = 0;

  /**
   * @brief Get the status velocities
   */
  virtual std::vector<float> getStatusVelocities() const = 0;

  /**
   * @brief Get the status positions
   */
  virtual std::vector<float> getStatusPositionsStatus() const = 0;

 protected:
  /**
   * @brief Constructor for the IFirmwareComm class
   */
  IFirmwareComm () = default;
  
 private:
  /**
   * @brief Receive data from the hardware
   */
  virtual bool receive() = 0;

  /**
   * @brief Send data to the hardware
   */
  virtual bool send() = 0;

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