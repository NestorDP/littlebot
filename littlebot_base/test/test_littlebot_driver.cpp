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

/**
 * @file test_firmware_comm.cpp
 * @brief Unit tests for LittlebotDriver class
 * @author Nestor Neto
 * @date 2024
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/types.hpp"
#include "littlebot_base/wheel.hpp"
#include "littlebot_base/codec.hpp"

#include "mock_serial_port.hpp"
#include "mock_rt_buffer.hpp"

// /**
//  * @brief Test fixture for LittlebotDriver tests
//  *
//  * This class provides common setup and teardown for LittlebotDriver tests.
//  */
class LittlebotDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    serial_ = std::make_shared<MockSerialPort>();
    state_buffer_ = std::make_shared<MockRTBuffer<littlebot_base::WheelRTData>>();
    command_buffer_ = std::make_shared<MockRTBuffer<littlebot_base::WheelRTData>>();

    driver = std::make_unique<littlebot_base::LittlebotDriver>(
      serial_, state_buffer_, command_buffer_, joint_names_);
  }

  std::shared_ptr<MockSerialPort> serial_;
  std::shared_ptr<MockRTBuffer<littlebot_base::WheelRTData>> state_buffer_;
  std::shared_ptr<MockRTBuffer<littlebot_base::WheelRTData>> command_buffer_;
  std::unique_ptr<littlebot_base::LittlebotDriver> driver;
  std::vector<std::string> joint_names_{"left_wheel", "right_wheel"};
};

TEST_F(LittlebotDriverTest, RequestStatusSuccess)
{
  std::string rx_payload;

  // Prepare a valid protobuf payload
  std::vector<littlebot_base::Wheel> wheels(2);
  wheels[0].setStatusVelocity(1.0f);
  wheels[0].setStatusPosition(2.0f);
  wheels[1].setStatusVelocity(3.0f);
  wheels[1].setStatusPosition(4.0f);

  // Encode the payload to a protobuf valid message
  littlebot_base::codec::encode(rx_payload, wheels);
  
  EXPECT_CALL(*serial_, write(std::string(1, littlebot_base::LittlebotDriver::kStatusChar)))
  .WillOnce(testing::Return(1));

  EXPECT_CALL(*serial_, read(testing::_))
  .WillOnce(testing::DoAll(
    testing::SetArgReferee<0>(rx_payload),
    testing::Return(rx_payload.size())));

  EXPECT_CALL(*state_buffer_, writeNonRT(testing::_))
  .WillOnce([](const littlebot_base::WheelRTData & data) {
      EXPECT_FLOAT_EQ(data.status_velocity[0], 1.0f);
      EXPECT_FLOAT_EQ(data.status_position[0], 2.0f);
      EXPECT_FLOAT_EQ(data.status_velocity[1], 3.0f);
      EXPECT_FLOAT_EQ(data.status_position[1], 4.0f);
      return true;
  });

  EXPECT_TRUE(driver->requestStatus());
  EXPECT_EQ(driver->getLastError(), littlebot_base::DriverError::None);
}
