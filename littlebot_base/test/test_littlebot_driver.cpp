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
    littlebot_driver_ = std::make_unique<littlebot_base::LittlebotDriver>(mock_serial_port_);
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
  std::shared_ptr<MockSerialPort> serial_port;
  std::unique_ptr<littlebot_base::LittlebotDriver> driver;

  serial_port = std::make_shared<MockSerialPort>();
  driver = std::make_unique<littlebot_base::LittlebotDriver>(serial_port);

  // Test that constructor successfully creates object with valid serial port
  ASSERT_NE(serial_port, nullptr);
  ASSERT_NE(driver, nullptr);

  driver.reset();
   serial_port.reset();
}

TEST(LittlebotDriverConstructorTest, ConstructorWithNullSerialPort)
{
  std::shared_ptr<littlebot_base::ISerialPort> null_port = nullptr;

  EXPECT_THROW({
    auto driver = std::make_unique<littlebot_base::LittlebotDriver>(null_port);
  }, std::invalid_argument);
}

TEST(LittlebotDriverConstructorTest, ConstructorWithDifferentSerialPorts)
{
  auto serial_port_1 = std::make_shared<MockSerialPort>();
  auto serial_port_2 = std::make_shared<MockSerialPort>();
  auto serial_port_3 = std::make_shared<MockSerialPort>();

  EXPECT_NO_THROW({
    auto driver_1 = std::make_unique<littlebot_base::LittlebotDriver>(serial_port_1);
    auto driver_2 = std::make_unique<littlebot_base::LittlebotDriver>(serial_port_2);
    auto driver_3 = std::make_unique<littlebot_base::LittlebotDriver>(serial_port_3);
  });
}

TEST(LittlebotDriverConstructorTest, ConstructorMemoryManagement)
{
  // Test that constructor properly manages shared_ptr reference counting
  {
    auto serial_port = std::make_shared<MockSerialPort>();
    auto initial_ref_count = serial_port.use_count();  // Should be 1

    {
      auto driver = std::make_unique<littlebot_base::LittlebotDriver>(serial_port);
      auto ref_count_after_construction = serial_port.use_count();  // Should be 2
      EXPECT_EQ(ref_count_after_construction, initial_ref_count + 1);
    }
    // After driver goes out of scope, ref count should decrease
    auto final_ref_count = serial_port.use_count();  // Should be 1 again
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

TEST_F(TestLittlebotDriver, ConstructorWithPreConfiguredSerialPort)
{
  // Create new LittlebotDriver with mock (mock has hardcoded response)
  auto serial_port = std::make_shared<MockSerialPort>();
  auto driver = std::make_unique<littlebot_base::LittlebotDriver>(serial_port);

  ASSERT_NE(driver, nullptr);

  // Test that it can receive data from the mock (hardcoded '[S.....]')
  uint8_t controller = driver->receiveData();
  std::cout << driver->getInputBuffer();  // For coverage
  EXPECT_EQ(controller, 'S');  // The hardcoded response starts with 'S'
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

// TEST_F(TestLittlebotDriver, SetCommandVelocities)
// {
//   // The littlebot_driver_ is automatically available and initialized
//   ASSERT_NE(littlebot_driver_, nullptr);

//   // Test setting valid velocities
//   std::vector<float> test_velocities = {1.5f, -2.3f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(test_velocities));

//   // Verify the values were stored correctly
//   auto retrieved_velocities = littlebot_driver_->getCommandVelocities();
//   EXPECT_EQ(retrieved_velocities.size(), 2u);
//   EXPECT_FLOAT_EQ(retrieved_velocities[0], 1.5f);
//   EXPECT_FLOAT_EQ(retrieved_velocities[1], -2.3f);

//   // Test with different velocity values
//   std::vector<float> zero_velocities = {0.0f, 0.0f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(zero_velocities));

//   // Verify the new values were stored
//   auto new_velocities = littlebot_driver_->getCommandVelocities();
//   EXPECT_EQ(new_velocities.size(), 2u);
//   EXPECT_FLOAT_EQ(new_velocities[0], 0.0f);
//   EXPECT_FLOAT_EQ(new_velocities[1], 0.0f);
// }

// TEST_F(TestLittlebotDriver, GetStatusVelocities)
// {
//   // The same littlebot_driver_ instance is available here too
//   ASSERT_NE(littlebot_driver_, nullptr);

//   // Get initial status velocities
//   std::vector<float> status_velocities = littlebot_driver_->getStatusVelocities();

//   // Should return a vector (might be empty initially)
//   ASSERT_GE(status_velocities.size(), 0u);

//   // Test that method doesn't throw
//   ASSERT_NO_THROW(littlebot_driver_->getStatusVelocities());
// }

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

// TEST_F(TestLittlebotDriver, SetCommandVelocities)
// {
//   // Test setting valid velocities
//   std::vector<float> test_velocities = {1.5f, -2.3f};

//   // This should not throw any exceptions
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(test_velocities));

//   // Test with different velocity values
//   std::vector<float> zero_velocities = {0.0f, 0.0f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(zero_velocities));

//   std::vector<float> positive_velocities = {5.0f, 3.2f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(positive_velocities));

//   std::vector<float> negative_velocities = {-1.8f, -4.1f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(negative_velocities));
// }

// TEST_F(TestLittlebotDriver, SetCommandVelocitiesDifferentSizes)
// {
//   // Test with empty vector
//   std::vector<float> empty_velocities = {};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(empty_velocities));

//   // Test with single element
//   std::vector<float> single_velocity = {1.0f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(single_velocity));

//   // Test with more than 2 elements
//   std::vector<float> multi_velocities = {1.0f, 2.0f, 3.0f, 4.0f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(multi_velocities));
// }

// TEST_F(TestLittlebotDriver, GetStatusVelocities)
// {
//   // Get initial status velocities
//   std::vector<float> status_velocities = littlebot_driver_->getStatusVelocities();

//   // Should return a vector (might be empty initially)
//   ASSERT_GE(status_velocities.size(), 0u);

//   // Test that method doesn't throw
//   ASSERT_NO_THROW(littlebot_driver_->getStatusVelocities());
// }

// TEST_F(TestLittlebotDriver, GetStatusPositions)
// {
//   // Get initial status positions
//   std::vector<float> status_positions = littlebot_driver_->getStatusPositionsStatus();

//   // Should return a vector (might be empty initially)
//   ASSERT_GE(status_positions.size(), 0u);

//   // Test that method doesn't throw
//   ASSERT_NO_THROW(littlebot_driver_->getStatusPositionsStatus());
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

// TEST_F(TestLittlebotDriver, ExtremeValues)
// {
//   // Test with very large positive values
//   std::vector<float> large_velocities = {1000.0f, 999.9f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(large_velocities));

//   // Test with very large negative values
//   std::vector<float> large_negative_velocities = {-1000.0f, -999.9f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(large_negative_velocities));

//   // Test with very small values
//   std::vector<float> small_velocities = {0.001f, -0.001f};
//   ASSERT_NO_THROW(littlebot_driver_->setCommandVelocities(small_velocities));
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
