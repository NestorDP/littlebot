// @ Copyright 2025-2026 Nestor Neto
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
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cstring>

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/types.hpp"
#include "littlebot_base/wheel.hpp"
#include "littlebot_base/codec.hpp"

#include "mock_serial_port.hpp"
#include "mock_rt_buffer.hpp"

/**
 * @brief Test fixture for LittlebotDriver tests
 *
 * This class provides common setup and teardown for LittlebotDriver tests.
 */
class LittlebotDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    serial_ = std::make_shared<MockSerialPort>();
    state_buffer_ = std::make_shared<MockRTBuffer<littlebot_base::WheelRTData>>();
    command_buffer_ = std::make_shared<MockRTBuffer<littlebot_base::WheelRTData>>();

    driver_ = std::make_unique<littlebot_base::LittlebotDriver>(
      serial_, state_buffer_, command_buffer_, joint_names_);
  }

  std::shared_ptr<MockSerialPort> serial_;
  std::shared_ptr<MockRTBuffer<littlebot_base::WheelRTData>> state_buffer_;
  std::shared_ptr<MockRTBuffer<littlebot_base::WheelRTData>> command_buffer_;
  std::unique_ptr<littlebot_base::LittlebotDriver> driver_;
  std::vector<std::string> joint_names_{"left_wheel", "right_wheel"};
};

TEST_F(LittlebotDriverTest, RequestStatusSuccess)
{
  std::vector<uint8_t> rx_payload;

  std::vector<littlebot_base::Wheel> wheels(2);
  wheels[0].setStatusVelocity(1.0f);
  wheels[0].setStatusPosition(2.0f);
  wheels[1].setStatusVelocity(3.0f);
  wheels[1].setStatusPosition(4.0f);

  littlebot_base::codec::encode(rx_payload, wheels);

  // Add control character at the beginning of the payload
  {
    std::vector<uint8_t> framed;
    framed.reserve(1 + rx_payload.size());
    framed.push_back(littlebot_base::LittlebotDriver::kStatusChar);
    framed.insert(framed.end(), rx_payload.begin(), rx_payload.end());
    rx_payload = std::move(framed);
  }

  EXPECT_CALL(*serial_, write(
    std::vector<uint8_t>(1, littlebot_base::LittlebotDriver::kStatusChar)))
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
  });

  EXPECT_TRUE(driver_->requestStatus());
  EXPECT_EQ(driver_->getLastError(), littlebot_base::DriverError::None);
}

TEST_F(LittlebotDriverTest, RequestStatusWriteFails)
{
  EXPECT_CALL(*serial_, write(testing::_))
  .WillOnce(testing::Return(0));

  EXPECT_FALSE(driver_->requestStatus());
}

TEST_F(LittlebotDriverTest, RequestStatusDecodeFailure)
{
  std::vector<uint8_t> garbage;
  garbage.reserve(1 + std::strlen("invalid_protobuf_payload"));
  garbage.push_back(littlebot_base::LittlebotDriver::kStatusChar);
  garbage.insert(garbage.end(),
    reinterpret_cast<const uint8_t *>("invalid_protobuf_payload"),
    reinterpret_cast<const uint8_t *>("invalid_protobuf_payload") +
    std::strlen("invalid_protobuf_payload"));

  EXPECT_CALL(*serial_, write(testing::_))
  .WillOnce(testing::Return(1));

  EXPECT_CALL(*serial_, read(testing::_))
  .WillOnce(testing::DoAll(
    testing::SetArgReferee<0>(garbage),
    testing::Return(garbage.size())));

  EXPECT_FALSE(driver_->requestStatus());
  EXPECT_EQ(driver_->getLastError(), littlebot_base::DriverError::DecodeFailure);
}

TEST_F(LittlebotDriverTest, RequestStatusControlCharFailure)
{
  std::vector<uint8_t> garbage =
  {'i', 'n', 'v', 'a', 'l', 'i', 'd', '_',
    'p', 'a', 'y', 'l', 'o', 'a', 'd', '_',
    'w', 'i', 't', 'h', 'o', 'u', 't', '_',
    'c', 'o', 'n', 't', 'r', 'o', 'l', '_',
    'c', 'h', 'a', 'r'};

  EXPECT_CALL(*serial_, write(testing::_))
  .WillOnce(testing::Return(1));

  EXPECT_CALL(*serial_, read(testing::_))
  .WillOnce(testing::DoAll(
    testing::SetArgReferee<0>(garbage),
    testing::Return(garbage.size())));

  EXPECT_FALSE(driver_->requestStatus());
  EXPECT_EQ(driver_->getLastError(), littlebot_base::DriverError::InvalidControlChar);
}

// TEST_F(LittlebotDriverTest, SendCommandSuccess)
// {
//   littlebot_base::WheelRTData cmd{};
//   cmd.command_velocity[0] = 5.0f;
//   cmd.command_velocity[1] = 6.0f;

//   EXPECT_CALL(*command_buffer_, readRT())
//   .WillOnce(testing::Return(&cmd));

//   EXPECT_CALL(*serial_, write(testing::_))
//   .WillOnce([](const std::vector<uint8_t> & payload) {
//       EXPECT_EQ(payload[0], littlebot_base::LittlebotDriver::kCommandChar);
//       return payload.size();
//   });

//   EXPECT_TRUE(driver_->sendCommand());
// }

TEST_F(LittlebotDriverTest, SendCommandNoRTData)
{
  EXPECT_CALL(*command_buffer_, readRT())
  .WillOnce(testing::Return(nullptr));

  EXPECT_FALSE(driver_->sendCommand());
  EXPECT_EQ(driver_->getLastError(), littlebot_base::DriverError::NoCommand);
}
