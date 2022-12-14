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

#include "littlebot_base/littlebot_talker.hpp"
#include "littlebot_base/littlebot_listener.hpp"
#include "littlebot_base/communication_protocol.hpp"

#include <libserial/serial.hpp>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  // comm::CommunicationProtocol s("/dev/rfcomm0"); 

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto talker   = std::make_shared<littlebot_base::Talker>(options);
  // auto listener = std::make_shared<littlebot_base::Listener>(options);

  // auto talker   = std::make_shared<littlebot_base::Talker>(&s, options);
  // auto listener = std::make_shared<littlebot_base::Listener>(&s, options);

  exec.add_node(talker);
  // exec.add_node(listener);
 
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
