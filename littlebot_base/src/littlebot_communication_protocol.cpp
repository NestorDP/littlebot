#include "littlebot_base/littlebot_communication_protocol.hpp"

namespace comm{ 
    
LittlebotCommunicationProtocol::LittlebotCommunicationProtocol(){}



LittlebotCommunicationProtocol::LittlebotCommunicationProtocol(std::string port): port_() {
  port_.OpenPort(port.c_str());
}

LittlebotCommunicationProtocol::~LittlebotCommunicationProtocol(){}


void LittlebotCommunicationProtocol::LittlebotRead(void){
  float *a, *b;
  port_.ReceiveMsg(&msg_protocol_);
  a = (float *) (msg_protocol_.c_str());
  b = (float *) (msg_protocol_.c_str() + 4);
  velocity_read_[0] = *a;
  velocity_read_[1] = *b;
  std::cout << msg_protocol_ << std::endl;
}

void LittlebotCommunicationProtocol::LittlebotWrite(void){
  unsigned int i;
  char vel_dir[sizeof(float)];
  char vel_lef[sizeof(float)];
  char *ptr;

  std::shared_ptr<std::string> msg_ptr;
  std::ostringstream msg;

  ptr = reinterpret_cast<char*>(&velocity_write_[0]);
  for (i = 0; i < sizeof(float); i++){
    vel_dir[i] = *(ptr + i);
  }

  ptr = reinterpret_cast<char*>(&velocity_write_[1]);
  for (i = 0; i < sizeof(float); i++){
    vel_lef[i] = *(ptr + i);
  }
  
  msg << vel_dir << '#' << vel_lef;
  port_.SendMsg(std::make_shared<std::string>(msg.str()));
}


void LittlebotCommunicationProtocol::SetVelocity(float dir, float lef) {
  velocity_write_[0] = dir;
  velocity_write_[1] = lef;
}


void LittlebotCommunicationProtocol::GetVelocity(float *dir, float *lef) {
  *dir = velocity_read_[0];
  *lef = velocity_read_[1];
}
}
