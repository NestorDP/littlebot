// //  @ Copyright 2024 Nestor Neto

#pragma once

#include <string>
#include <vector>

#include <libserial/serial.hpp>

#include "littlebot_base/littlebot_communication_interface.hpp"

namespace littlebot_base
{

class LittlebotCommunication : public LittlebotCommunicationInterface
{
public:
  /**
   * @brief Constructor for the LittlebotCommunication class
   */
  LittlebotCommunication(const std::string serial_port);
  
  /**
   * @brief Deconstructor for the LittlebotCommunication class
   */
  ~LittlebotCommunication();

  /**
   * @brief Set the command velocities
   */
  void setCommandVelocities([[maybe_unused]] std::vector<float> velocities) override;

  /**
   * @brief Get the status velocities
   */
  std::vector<float> getStatusVelocities() const override;

  /**
   * @brief Get the status positions
   */
  std::vector<float> getStatusPositionsStatus() const override;

  /**
   * @brief Start threads
   * 
   * This function starts the threads that read and write data to the hardware.
   */
  bool start();

  /**
   * @brief Stop thread that read data from serial port
   * 
   * This function stops the threads that read and write data to the hardware.
   */
  bool stop();

private:
  /**
   * @brief Receive data from the hardware
   * 
   * This function receives the status positions and velocities from the hardware.
   */
  bool receive() override;

  /**
   * @brief Send data to the hardware
   * 
   * This function sends the command velocities to the hardware.
   */
  bool send() override;

  /**
   * @brief Command velocities for the hardware.
   * 
   * This vector stores the command velocities that are sent to the hardware.
   */
  std::vector<float> command_velocities_;

  /**
   * @brief Status positions from the hardware.
   * 
   * This vector stores the status positions received from the hardware.
   */
  std::vector<float> status_positions_;

  /**
   * @brief Status velocities from the hardware.
   * 
   * This vector stores the status velocities received from the hardware.
   */
  std::vector<float> status_velocities_;

  /**
   * @brief Smart pointer to serial port object
   * 
   * This object is used to communicate with the hardware. To more information 
   * about the serial library used, please visit: https://github.com/NestorDP/libserial
   */
  std::shared_ptr<serial::Serial> serial_;

  /**
   * @brief Caracter to start the message
   */
  static constexpr char kStartByte{0x3C};

  /**
   * @brief Caracter to end the message
   */
  static constexpr char kEndByte{0x3E};


};

}  // namespace littlebot_base