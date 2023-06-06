// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "littlebot_base/reader.hpp"
#include "littlebot_base/writer.hpp"
#include "littlebot_base/protocol.hpp"

#include <libserial/serial.hpp>

int main(int argc, char * argv[])
{
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
