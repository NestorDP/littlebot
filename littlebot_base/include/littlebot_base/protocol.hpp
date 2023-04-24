//  @ Copyright 2022 Nestor Neto

/**
 * @mainpage Serial Interface Library
 * @section intro_sec Indroduction
 * This is an interface C++ library for serial serial
 * 
 * 
 */
#ifndef _BASE__COMMUNICATION_PROTOCOL_HPP_
#define _BASE__COMMUNICATION_PROTOCOL_HPP_

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include <libserial/serial.hpp>


namespace littlebot{

class Protocol {
 public:
  Protocol();
  /**
   * @brief Constructor of the serial class
   * 
   */
  Protocol(std::string port);

  /**
   * @brief Destroyer of the serial class
   * 
   */
  ~Protocol();

  /**
   * @brief
   * 
   */
  void Read(void);

  /**
   * @brief
   * 
   */
  void Write(void);

  /**
   * @brief
   * 
   */
  void SetVelocity(float dir, float lef);

    /**
   * @brief
   * 
   */
  void GetVelocity(float *dir, float *lef);


 private:
  /** Length of buffer */
  const int kLengthBuffer_ = 200;

  /** Port */
  serial::Serial port_ ;

  /** Buffer */
  std::string msg_protocol_;

  /**  */
  float velocity_read_[2];
  float velocity_write_[2];

};
}  // namespace serial

#endif  // _BASE__COMMUNICATION_PROTOCOL_HPP_
