 #include "littlebot_base/littlebot_communication_protocol.hpp"

comm::LittlebotCommunicationProtocol::LittlebotCommunicationProtocol(std::string port){
  port_.open_port(port.c_str());
}

void comm::LittlebotCommunicationProtocol::ReceiveMsg(void){
  port_.receive_msg(&msg_protocol);
  
}
