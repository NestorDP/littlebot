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



class TestLittlebotHardwareComponent : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Do not overwrite the existing URDF; use the file already present at urdf_file_path_
  }

  std::string urdf_file_path_{"/home/nestor/littlebot_ws/build/littlebot_base/test_littlebot.urdf"};
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
  auto hardware_info = control_resources.front();

  // Check parameters
  // In this ROS2 / hardware_interface version parameters are on HardwareInfo directly
  const auto & hw_info_params = hardware_info.hardware_parameters;

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info;

  // Check that parameters were read
  EXPECT_GT(hw_info_params.size(), 0u);

  // Print all parameters key = value
  std::cout << "Successfully read parameters from URDF! (" << hw_info_params.size() << " entries)" << std::endl;
  for (const auto & kv : hw_info_params) {
    std::cout << "  " << kv.first << " = " << kv.second << std::endl;
  }

  // Initialize LittlebotHardwareComponent
  auto littlebot_hardware = std::make_shared<littlebot_base::LittlebotHardwareComponent>();
  auto ret = littlebot_hardware->on_init(params);
  EXPECT_EQ(
    ret,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  ) << "Failed to initialize LittlebotHardwareComponent from URDF parameters";

  in.close();
}
