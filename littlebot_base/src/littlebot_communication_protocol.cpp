 #include "littlebot_base/littlebot_communication_protocol.hpp"

comm::LittlebotCommunicationProtocol::LittlebotCommunicationProtocol(std::string port): port_() {
  port_.OpenPort(port.c_str());
  // velocity_write_[0] = 2.14;
  // velocity_write_[1] = 3.14;
}

void comm::LittlebotCommunicationProtocol::LittlebotRead(void){
  port_.ReceiveMsg(&msg_protocol_);

  std::cout << msg_protocol_ << std::endl;
}

void comm::LittlebotCommunicationProtocol::LittlebotWrite(void){
  unsigned int i;
  char vel_dir[sizeof(float)];
  char vel_lef[sizeof(float)];
  char *ptr;

  float *a, *b;

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

  a = (float*) &vel_dir[0];
  b = (float*) &vel_lef[0];

  printf("%.2f#%.2f\n", *a, *b);

  msg << vel_dir << '#' << vel_lef;

  port_.SendMsg(std::make_shared<std::string>(msg.str()));
}


void comm::LittlebotCommunicationProtocol::SetVelocity(float *dir, float *lef) {
  velocity_write_[0] = *dir;
  velocity_write_[1] = *lef;
}

void comm::LittlebotCommunicationProtocol::GetVelocity(float *dir, float *lef) {
  *dir = velocity_read_[0];
  *lef = velocity_read_[1];
}



// SEND FLOAT VARIABLE OVER UART ALGORITHM
// int main()
// {
//     float f = 4.7838322;
//     unsigned char *ptr, i;
//     float *a;    
//     unsigned char var[sizeof(float)];
//     ptr = (unsigned char *)& f;
//     for (i = 0; i < sizeof(float); i++){
//         var[i] = *(ptr + i);
//     }
//     printf("Valores float dividido em bytes %s", var);
//     a = (float*) &var[0];
//     printf("Valores float para um ponteiro array char %f", *a);
//     return 0;
// }
