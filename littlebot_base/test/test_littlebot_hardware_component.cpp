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
    mock_serial_port_ = std::make_shared<MockSerialPort>();
    littlebot_hardware_component_ =
      std::make_shared<littlebot_base::LittlebotHardwareComponent>();
  }

  void TearDown() override
  {
    littlebot_hardware_component_.reset();
    // mock_serial_port_.reset();
  }

  std::shared_ptr<littlebot_base::ISerialPort> mock_serial_port_;

  std::shared_ptr<littlebot_base::LittlebotHardwareComponent>
  littlebot_hardware_component_;

  hardware_interface::HardwareInfo hardware_info_;

  const std::string urdf_file_path_{
    "/home/nestor/littlebot_ws/build/littlebot_base/test_littlebot.urdf"};
};

TEST_F(TestLittlebotHardwareComponent, InitializeFromURDF)
{
  // Read the existing URDF file and print its contents for debugging
  std::ifstream in(urdf_file_path_);
  ASSERT_TRUE(in.is_open()) << "Failed to open existing URDF file: " << urdf_file_path_;

  // Read the entire file content into a string
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string urdf_content = ss.str();

  // Parse URDF
  auto control_resources = hardware_interface::parse_control_resources_from_urdf(urdf_content);
  hardware_info_ = control_resources.front();

  // Check parameters
  const auto & hw_info_params = hardware_info_.hardware_parameters;

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;

  // Check that parameters were read
  EXPECT_GT(hw_info_params.size(), 0u);

  auto ret = littlebot_hardware_component_->on_init(params);
  EXPECT_EQ(
    ret,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  ) << "Failed to initialize LittlebotHardwareComponent from URDF parameters";

  littlebot_hardware_component_->setupDriver(
    mock_serial_port_, "/dev/hwcom", 1000);

  in.close();
}

TEST_F(TestLittlebotHardwareComponent, InterfacesAreExportedCorrectly)
{
  // Export interfaces
  auto state_interfaces = littlebot_hardware_component_->export_state_interfaces();
  auto command_interfaces = littlebot_hardware_component_->export_command_interfaces();

  // Check that the correct number of interfaces are exported
  EXPECT_EQ(state_interfaces.size(), hardware_info_.joints.size() * 2)
    << "Unexpected number of state interfaces exported";
  EXPECT_EQ(command_interfaces.size(), hardware_info_.joints.size())
    << "Unexpected number of command interfaces exported";

  // Check that the interfaces have the correct names and types
  for (size_t i = 0; i < hardware_info_.joints.size(); ++i) {
    const auto & joint_name = hardware_info_.joints[i].name;

    // State interfaces
    EXPECT_EQ(state_interfaces[i * 2].get_name(), joint_name)
      << "State interface name mismatch for joint: " << joint_name;
    EXPECT_EQ(state_interfaces[i * 2].get_interface_name(), hardware_interface::HW_IF_POSITION)
      << "State interface type mismatch for joint: " << joint_name;

    EXPECT_EQ(state_interfaces[i * 2 + 1].get_name(), joint_name)
      << "State interface name mismatch for joint: " << joint_name;
    EXPECT_EQ(state_interfaces[i * 2 + 1].get_interface_name(), hardware_interface::HW_IF_VELOCITY)
      << "State interface type mismatch for joint: " << joint_name;
  }

  // Command interfaces
  for (size_t i = 0; i < hardware_info_.joints.size(); ++i) {
    const auto & joint_name = hardware_info_.joints[i].name;
    EXPECT_EQ(command_interfaces[i].get_name(), joint_name)
      << "Command interface name mismatch for joint: " << joint_name;
    EXPECT_EQ(command_interfaces[i].get_interface_name(), hardware_interface::HW_IF_POSITION)
      << "Command interface type mismatch for joint: " << joint_name;
  }
}
