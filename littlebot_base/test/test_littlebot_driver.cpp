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
#include "mock_serial_port.hpp"

/**
 * @brief Test fixture for LittlebotDriver tests
 *
 * This class provides common setup and teardown for LittlebotDriver tests.
 */
class TestLittlebotDriver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mock_serial_port_ = std::make_shared<MockSerialPort>();
    littlebot_driver_ = std::make_unique<littlebot_base::LittlebotDriver>(
      mock_serial_port_, "/dev/ttyUSB0", 115200);
  }

  void TearDown() override
  {
    littlebot_driver_.reset();
    mock_serial_port_.reset();
  }

  std::shared_ptr<MockSerialPort> mock_serial_port_;
  std::unique_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;
};

TEST(LittlebotDriverConstructorTest, ConstructorWithValidSerialPort)
{
  std::shared_ptr<MockSerialPort> mock_serial_port;
  std::unique_ptr<littlebot_base::LittlebotDriver> driver;

  mock_serial_port = std::make_shared<MockSerialPort>();
  driver = std::make_unique<littlebot_base::LittlebotDriver>(
      mock_serial_port, "/dev/ttyUSB0", 115200);

  // Test that constructor successfully creates object with valid serial port
  ASSERT_NE(mock_serial_port, nullptr);
  ASSERT_NE(driver, nullptr);

  driver.reset();
  mock_serial_port.reset();
}

TEST(LittlebotDriverConstructorTest, ConstructorWithNullSerialPort)
{
  std::shared_ptr<littlebot_base::ISerialPort> null_port = nullptr;

  EXPECT_THROW({
    auto driver = std::make_unique<littlebot_base::LittlebotDriver>(
      null_port, "/dev/ttyUSB0", 115200);
  }, std::invalid_argument);
}

TEST(LittlebotDriverConstructorTest, ConstructorWithDifferentSerialPorts)
{
  auto mock_serial_port_1 = std::make_shared<MockSerialPort>();
  auto mock_serial_port_2 = std::make_shared<MockSerialPort>();
  auto mock_serial_port_3 = std::make_shared<MockSerialPort>();

  EXPECT_NO_THROW({
    auto driver_1 = std::make_unique<littlebot_base::LittlebotDriver>(
      mock_serial_port_1, "/dev/ttyUSB0", 115200);
    auto driver_2 = std::make_unique<littlebot_base::LittlebotDriver>(
      mock_serial_port_2, "/dev/ttyUSB1", 115200);
    auto driver_3 = std::make_unique<littlebot_base::LittlebotDriver>(
      mock_serial_port_3, "/dev/ttyUSB2", 115200);
  });
}

TEST(LittlebotDriverConstructorTest, ConstructorMemoryManagement)
{
  // Test that constructor properly manages shared_ptr reference counting
  {
    auto mock_serial_port = std::make_shared<MockSerialPort>();
    auto initial_ref_count = mock_serial_port.use_count();  // Should be 1

    {
      auto driver = std::make_unique<littlebot_base::LittlebotDriver>(
      mock_serial_port, "/dev/ttyUSB0", 115200);
      auto ref_count_after_construction = mock_serial_port.use_count();  // Should be 2
      EXPECT_EQ(ref_count_after_construction, initial_ref_count + 1);
    }
    // After driver goes out of scope, ref count should decrease
    auto final_ref_count = mock_serial_port.use_count();  // Should be 1 again
    EXPECT_EQ(final_ref_count, initial_ref_count);
  }
}

TEST_F(TestLittlebotDriver, InitialStateAfterConstruction)
{
  auto input_buffer = littlebot_driver_->getInputBuffer();
  EXPECT_TRUE(input_buffer->empty());

  // Status velocities should return initial values (implementation dependent)
  EXPECT_NO_THROW(littlebot_driver_->getStatusVelocities());
  EXPECT_NO_THROW(littlebot_driver_->getStatusPositions());
}

// TEST(LittlebotDriverConstructorTest, ConstructorExceptionSafety)
// {
//   // Test that constructor properly throws and doesn't leak memory
//   std::shared_ptr<littlebot_base::ISerialPort> null_port = nullptr;

//   try {
//     auto firmware = std::make_unique<littlebot_base::LittlebotDriver>(null_port);
//     FAIL() << "Expected std::invalid_argument exception";
//   } catch (const std::invalid_argument & e) {
//     // Expected exception
//     EXPECT_STREQ(e.what(), "Serial port cannot be null");
//   } catch (...) {
//     FAIL() << "Expected std::invalid_argument, got different exception";
//   }
// }

TEST_F(TestLittlebotDriver, SetCommandVelocities)
{
  std::map<std::string, float> test_velocities;
  auto wheel_names = littlebot_driver_->getWheelNames();
  ASSERT_EQ(wheel_names.size(), 2u);

  test_velocities[wheel_names[0]] = 0.0f;
  test_velocities[wheel_names[1]] = 0.0f;
  ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(test_velocities));

  test_velocities[wheel_names[0]] = 1.5f;
  test_velocities[wheel_names[1]] = 2.3f;
  ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(test_velocities));

  test_velocities[wheel_names[0]] = -1.5f;
  test_velocities[wheel_names[1]] = -2.3f;
  ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(test_velocities));
}

TEST_F(TestLittlebotDriver, GetStatusVelocities)
{
  // Get initial status velocities
  std::map<std::string, float> status_velocities;
  ASSERT_NO_THROW(status_velocities = littlebot_driver_->getStatusVelocities());

  ASSERT_GE(status_velocities["left"], 0.0f);
  ASSERT_GE(status_velocities["right"], 0.0f);
}

TEST_F(TestLittlebotDriver, GetStatusPositions)
{
  // Get initial status positions
  std::map<std::string, float> status_positions;
  ASSERT_NO_THROW(status_positions = littlebot_driver_->getStatusPositions());

  // Should return a vector (might be empty initially)
  ASSERT_GE(status_positions["left"], 0.0f);
  ASSERT_GE(status_positions["right"], 0.0f);
}

TEST_F(TestLittlebotDriver, ReceiveDataFromSerialPort)
{
  char controller_char;
  ASSERT_NO_THROW(controller_char = littlebot_driver_->receiveData());

  // Check that controller character is as expected
  ASSERT_EQ(controller_char, 'S');

  // Check that input buffer is not empty after receiving data
  auto input_buffer = littlebot_driver_->getInputBuffer();
  ASSERT_FALSE(input_buffer->empty());
}

TEST_F(TestLittlebotDriver, SendDataToSerialPort)
{
  // The same littlebot_driver_ instance is available here too
  ASSERT_NE(littlebot_driver_, nullptr);

  // Test that method doesn't throw
  ASSERT_NO_THROW(littlebot_driver_->sendData('C'));
  ASSERT_NO_THROW(littlebot_driver_->sendData('S'));
  auto output_buffer = littlebot_driver_->getOutputBuffer();
  ASSERT_FALSE(output_buffer->empty());
}

TEST_F(TestLittlebotDriver, SendCommandData)
{
  std::map<std::string, float> test_velocities;
  auto wheel_names = littlebot_driver_->getWheelNames();
  ASSERT_EQ(wheel_names.size(), 2u);

  test_velocities[wheel_names[0]] = 1.5f;
  test_velocities[wheel_names[1]] = -2.3f;
  littlebot_driver_->setCommandVelocities(test_velocities);

  // Send command data
  ASSERT_NO_THROW(littlebot_driver_->sendData('C'));

  // Check that output buffer contains the correct command data
  auto output_buffer = littlebot_driver_->getOutputBuffer();
  ASSERT_FALSE(output_buffer->empty());
}

TEST_F(TestLittlebotDriver, ReceiveStatusData)
{
  // Receive status data
  char controller_char;
  ASSERT_NO_THROW(controller_char = littlebot_driver_->receiveData());
  ASSERT_EQ(controller_char, 'S');

  // Check that status velocities and positions are updated
  std::map<std::string, float> status_velocities;
  std::map<std::string, float> status_positions;

  ASSERT_NO_THROW(status_velocities = littlebot_driver_->getStatusVelocities());
  ASSERT_NO_THROW(status_positions = littlebot_driver_->getStatusPositions());

  std::cout << "Status Velocities: left=" << status_velocities["left_wheel"]
            << ", right=" << status_velocities["right_wheel"] << std::endl;
  std::cout << "Status Positions: left=" << status_positions["left_wheel"]
            << ", right=" << status_positions["right_wheel"] << std::endl;

  EXPECT_EQ(status_velocities["left_wheel"], 4.56f);
  EXPECT_EQ(status_velocities["right_wheel"], 5.67f);
  EXPECT_EQ(status_positions["left_wheel"], 7.89f);
  EXPECT_EQ(status_positions["right_wheel"], 8.90f);
}


// TEST_F(TestLittlebotDriver, ConstructorDestructor)
// {
//   // Test that constructor creates object successfully
//   ASSERT_NE(littlebot_driver_, nullptr);

//   // Test with different serial port names
//   auto comm1 = std::make_unique<MockableLittlebotDriver>("/dev/ttyUSB0");
//   ASSERT_NE(comm1, nullptr);

//   auto comm2 = std::make_unique<MockableLittlebotDriver>("/dev/ttyUSB1");
//   ASSERT_NE(comm2, nullptr);

//   auto comm3 = std::make_unique<MockableLittlebotDriver>("/dev/ttyACM0");
//   ASSERT_NE(comm3, nullptr);
// }

// TEST_F(TestLittlebotDriver, DataConsistency)
// {
//   // Set some test velocities
//   std::vector<float> test_velocities = {2.5f, -1.8f};
//   littlebot_driver_->setCommandVelocities(test_velocities);

//   // Verify that the object can handle multiple operations
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(test_velocities));
//   ASSERT_NO_THROW(littlebot_driver_->getStatusVelocities());
//   ASSERT_NO_THROW(littlebot_driver_->getStatusPositionsStatus());
// }

// TEST_F(TestLittlebotDriver, InterfaceCompliance)
// {
//   // Test that LittlebotDriver can be used through its interface
//   std::unique_ptr<ILittlebotDriver> interface_ptr =
//     std::make_unique<MockableLittlebotDriver>("/dev/null");

//   ASSERT_NE(interface_ptr, nullptr);

//   // Test interface methods
//   std::vector<float> test_velocities = {1.0f, 2.0f};
//   ASSERT_NO_THROW(interface_ptr->setCommandVelocities(test_velocities));
//   ASSERT_NO_THROW(interface_ptr->getStatusVelocities());
//   ASSERT_NO_THROW(interface_ptr->getStatusPositionsStatus());
// }

// TEST_F(TestLittlebotDriver, RapidCalls)
// {
//   const int num_iterations = 100;

//   for (int i = 0; i < num_iterations; ++i) {
//     std::vector<float> velocities = {
//       static_cast<float>(i % 10),
//       static_cast<float>((i + 1) % 10)
//     };

//     ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(velocities));
//     ASSERT_NO_THROW(littlebot_driver_->getStatusVelocities());
//     ASSERT_NO_THROW(littlebot_driver_->getStatusPositionsStatus());
//   }
// }

// TEST_F(TestLittlebotDriver, MemoryManagement)
// {
//   // Test creating and destroying multiple instances
//   const int num_instances = 10;
//   std::vector<std::unique_ptr<MockableLittlebotDriver>> instances;

//   // Create multiple instances
//   for (int i = 0; i < num_instances; ++i) {
//     auto instance = std::make_unique<MockableLittlebotDriver>("/dev/null");
//     ASSERT_NE(instance, nullptr);
//     instances.push_back(std::move(instance));
//   }

//   // Use all instances
//   for (auto & instance : instances) {
//     std::vector<float> test_velocities = {1.0f, 2.0f};
//     ASSERT_NO_THROW(instance->setCommandVelocities(test_velocities));
//   }

//   // Instances will be automatically destroyed when vector goes out of scope
// }

// TEST_F(TestLittlebotDriver, BasicPerformance)
// {
//   const int num_operations = 1000;
//   std::vector<float> test_velocities = {1.5f, -2.3f};

//   // Time the operations (basic check that they complete in reasonable time)
//   auto start = std::chrono::high_resolution_clock::now();

//   for (int i = 0; i < num_operations; ++i) {
//     littlebot_driver_->setCommandVelocities(test_velocities);
//     littlebot_driver_->getStatusVelocities();
//     littlebot_driver_->getStatusPositionsStatus();
//   }

//   auto end = std::chrono::high_resolution_clock::now();
//   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

//   // Operations should complete within reasonable time (less than 1 second for 1000 operations)
//   ASSERT_LT(duration.count(), 1000);
// }
