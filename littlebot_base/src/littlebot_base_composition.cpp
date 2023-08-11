//  @ Copyright 2023 Nestor Neto

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "littlebot_base/reader.hpp"
#include "littlebot_base/writer.hpp"
#include "littlebot_base/protocol.hpp"

#include <libserial/serial.hpp>

int main(int argc, char * argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  serial::Serial ser;
  ser.OpenPort("/dev/rfcomm0");
  ser.SetFlowControl(serial::FlowControl::Software);

  // auto reader = std::make_shared<littlebot_base::Reader>(options, &ser);
  auto writer = std::make_shared<littlebot_base::Writer>(options, &ser);

  // exec.add_node(reader);
  exec.add_node(writer);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
