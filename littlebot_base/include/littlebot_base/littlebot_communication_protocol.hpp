//  @ Copyright 2022 Nestor Neto

/**
 * @mainpage Serial Interface Library
 * @section intro_sec Indroduction
 * This is an interface C++ library for serial serial
 * 
 * 
 */
#ifndef LITTLEBOT_BASE_LITTLEBOT_COMMUNICATION_PROTOCOL_HPP_
#define LITTLEBOT_BASE_LITTLEBOT_COMMUNICATION_PROTOCOL_HPP_

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <libserial/serial.hpp>


namespace comm {

class LittlebotCommunicationProtocol {
 public:
  /**
   * @brief Constructor of the serial class
   * 
   */
  LittlebotCommunicationProtocol(std::string port);

  /**
   * @brief Destroyer of the serial class
   * 
   */
  ~LittlebotCommunicationProtocol();

  /**
   * @brief Method to open the serial communication
   * 
   */
  void ReceiveMsg(void);


 private:
   /** Length of buffer */
  const int kLengthBuffer_ = 200;

  /** Port */
  serial::Serial port_;

  /** Buffer */
  std::string msg_protocol;

  std::array<float, 2> velocity_read;
  std::array<float, 2> velocity_write;

};
}  // namespace serial

#endif  // LITTLEBOT_BASE_LITTLEBOT_COMMUNICATION_PROTOCOL_HPP_
