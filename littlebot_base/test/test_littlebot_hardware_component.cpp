// @ Copyright 2025 Nestor Neto
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <filesystem>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/littlebot_hardware_component.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "mock_serial_port.hpp"

class TestLittlebotHardwareComponent : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<littlebot_base::LittlebotHardwareComponent>();
    params_ = makeValidParams();
  }

  hardware_interface::HardwareComponentInterfaceParams makeValidParams()
  {
    hardware_interface::HardwareComponentInterfaceParams params;
    auto & info = params.hardware_info;

    info.name = "littlebot";
    info.type = "system";

    info.joints.push_back(makeJoint("left_wheel_joint"));
    info.joints.push_back(makeJoint("right_wheel_joint"));

    info.hardware_parameters["serial_port"] = "/dev/ttyUSB_FAKE";
    info.hardware_parameters["baudrate"] = "115200";

    return params;
  }

  hardware_interface::ComponentInfo makeJoint(const std::string & name)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = name;

    hardware_interface::InterfaceInfo cmd_interface;
    cmd_interface.name = hardware_interface::HW_IF_VELOCITY;
    cmd_interface.size = 1;
    cmd_interface.enable_limits = false;
    joint.command_interfaces.push_back(cmd_interface);

    hardware_interface::InterfaceInfo pos_interface;
    pos_interface.name = hardware_interface::HW_IF_POSITION;
    pos_interface.size = 1;
    pos_interface.enable_limits = false;
    joint.state_interfaces.push_back(pos_interface);

    hardware_interface::InterfaceInfo vel_interface;
    vel_interface.name = hardware_interface::HW_IF_VELOCITY;
    vel_interface.size = 1;
    vel_interface.enable_limits = false;
    joint.state_interfaces.push_back(vel_interface);

    return joint;
  }

  std::unique_ptr<littlebot_base::LittlebotHardwareComponent> hw_;
  hardware_interface::HardwareComponentInterfaceParams params_;
};

TEST_F(TestLittlebotHardwareComponent, InitSucceedsWithValidParams)
{
  auto params = makeValidParams();

  auto ret = hw_->on_init(params);

  EXPECT_EQ(
    ret,
    hardware_interface::CallbackReturn::SUCCESS
  );
}
