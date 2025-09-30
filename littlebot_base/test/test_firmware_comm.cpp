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
 * @brief Unit tests for FirmwareComm class
 * @author Nestor Neto
 * @date 2024
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "littlebot_base/firmware_comm.hpp"
#include "mock_serial_port.hpp"

/**
 * @brief Test fixture for FirmwareComm tests
 *
 * This class provides common setup and teardown for FirmwareComm tests.
 */
class FirmwareCommTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a test instance with a mock serial port
    mock_serial_port_ = std::make_shared<MockSerialPort>("/dev/ttyUSB0", 115200);
    firmware_comm_ = std::make_unique<littlebot_base::FirmwareComm>(mock_serial_port_);
  }

  void TearDown() override
  {
    // Clean up
    firmware_comm_.reset();
    mock_serial_port_.reset();
  }

  // Test instances
  std::shared_ptr<MockSerialPort> mock_serial_port_;
  std::unique_ptr<littlebot_base::FirmwareComm> firmware_comm_;
};

/**
 * @brief Test FirmwareComm constructor with valid serial port
 */
TEST_F(FirmwareCommTest, ConstructorWithValidSerialPort)
{
  // Test that constructor successfully creates object with valid serial port
  ASSERT_NE(firmware_comm_, nullptr);
  ASSERT_NE(mock_serial_port_, nullptr);
  
  // Test that the object is properly initialized
  EXPECT_NO_THROW(firmware_comm_->getInputBuffer());
}

/**
 * @brief Test FirmwareComm constructor with null serial port
 */
TEST(FirmwareCommConstructorTest, ConstructorWithNullSerialPort)
{
  // Test that constructor throws exception with null serial port
  std::shared_ptr<littlebot_base::ISerialPort> null_port = nullptr;
  
  EXPECT_THROW({
    auto firmware = std::make_unique<littlebot_base::FirmwareComm>(null_port);
  }, std::invalid_argument);
}

/**
 * @brief Test FirmwareComm constructor with different serial port implementations
 */
TEST(FirmwareCommConstructorTest, ConstructorWithDifferentSerialPorts)
{
  // Test with multiple mock serial ports
  auto mock1 = std::make_shared<MockSerialPort>();
  auto mock2 = std::make_shared<MockSerialPort>();
  auto mock3 = std::make_shared<MockSerialPort>();
  
  // All should construct successfully
  EXPECT_NO_THROW({
    auto firmware1 = std::make_unique<littlebot_base::FirmwareComm>(mock1);
    auto firmware2 = std::make_unique<littlebot_base::FirmwareComm>(mock2);
    auto firmware3 = std::make_unique<littlebot_base::FirmwareComm>(mock3);
  });
}

/**
 * @brief Test initial state after construction
 */
TEST_F(FirmwareCommTest, InitialStateAfterConstruction)
{  
  // Input buffer should be empty initially
  auto input_buffer = firmware_comm_->getInputBuffer();
  EXPECT_TRUE(input_buffer.empty());
  
  // Status velocities should return initial values (implementation dependent)
  EXPECT_NO_THROW(firmware_comm_->getStatusVelocities());
  EXPECT_NO_THROW(firmware_comm_->getStatusPositions());
}

/**
 * @brief Test constructor memory management
 */
TEST(FirmwareCommConstructorTest, ConstructorMemoryManagement)
{
  // Test that constructor properly manages shared_ptr reference counting
  {
    auto mock_port = std::make_shared<MockSerialPort>();
    auto initial_ref_count = mock_port.use_count(); // Should be 1
    
    {
      auto firmware = std::make_unique<littlebot_base::FirmwareComm>(mock_port);
      auto ref_count_after_construction = mock_port.use_count(); // Should be 2
      EXPECT_EQ(ref_count_after_construction, initial_ref_count + 1);
    }
    // After firmware goes out of scope, ref count should decrease
    auto final_ref_count = mock_port.use_count(); // Should be 1 again
    EXPECT_EQ(final_ref_count, initial_ref_count);
  }
}

/**
 * @brief Test constructor with serial port that has pre-configured state
 */
TEST_F(FirmwareCommTest, ConstructorWithPreConfiguredSerialPort)
{
  // Create new FirmwareComm with mock (mock has hardcoded response)
  auto new_mock = std::make_shared<MockSerialPort>();
  auto firmware = std::make_unique<littlebot_base::FirmwareComm>(new_mock);
  
  ASSERT_NE(firmware, nullptr);
  
  // Test that it can receive data from the mock (hardcoded '[S.....]')
  uint8_t controller = firmware->receiveData();
  EXPECT_EQ(controller, 'S'); // The hardcoded response starts with 'S'
}

/**
 * @brief Test constructor exception safety
 */
TEST(FirmwareCommConstructorTest, ConstructorExceptionSafety)
{
  // Test that constructor properly throws and doesn't leak memory
  std::shared_ptr<littlebot_base::ISerialPort> null_port = nullptr;
  
  try {
    auto firmware = std::make_unique<littlebot_base::FirmwareComm>(null_port);
    FAIL() << "Expected std::invalid_argument exception";
  } catch (const std::invalid_argument& e) {
    // Expected exception
    EXPECT_STREQ(e.what(), "Serial port cannot be null");
  } catch (...) {
    FAIL() << "Expected std::invalid_argument, got different exception";
  }
}

/**
 * @brief Test setting command velocities (using same fixture)
 */
// TEST_F(FirmwareCommTest, SetCommandVelocities)
// {
//   // The firmware_comm_ is automatically available and initialized
//   ASSERT_NE(firmware_comm_, nullptr);
  
//   // Test setting valid velocities
//   std::vector<float> test_velocities = {1.5f, -2.3f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(test_velocities));
  
//   // Verify the values were stored correctly
//   auto retrieved_velocities = firmware_comm_->getCommandVelocities();
//   EXPECT_EQ(retrieved_velocities.size(), 2u);
//   EXPECT_FLOAT_EQ(retrieved_velocities[0], 1.5f);
//   EXPECT_FLOAT_EQ(retrieved_velocities[1], -2.3f);
  
//   // Test with different velocity values
//   std::vector<float> zero_velocities = {0.0f, 0.0f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(zero_velocities));
  
//   // Verify the new values were stored
//   auto new_velocities = firmware_comm_->getCommandVelocities();
//   EXPECT_EQ(new_velocities.size(), 2u);
//   EXPECT_FLOAT_EQ(new_velocities[0], 0.0f);
//   EXPECT_FLOAT_EQ(new_velocities[1], 0.0f);
// }

/**
 * @brief Test getting status velocities (using same fixture)
 */
// TEST_F(FirmwareCommTest, GetStatusVelocities)
// {
//   // The same firmware_comm_ instance is available here too
//   ASSERT_NE(firmware_comm_, nullptr);
  
//   // Get initial status velocities
//   std::vector<float> status_velocities = firmware_comm_->getStatusVelocities();
  
//   // Should return a vector (might be empty initially)
//   ASSERT_GE(status_velocities.size(), 0u);
  
//   // Test that method doesn't throw
//   ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
// }

/**
 * @brief Test constructor and destructor
 */
// TEST_F(FirmwareCommTest, ConstructorDestructor)
// {
//   // Test that constructor creates object successfully
//   ASSERT_NE(firmware_comm_, nullptr);

//   // Test with different serial port names
//   auto comm1 = std::make_unique<MockableFirmwareComm>("/dev/ttyUSB0");
//   ASSERT_NE(comm1, nullptr);

//   auto comm2 = std::make_unique<MockableFirmwareComm>("/dev/ttyUSB1");
//   ASSERT_NE(comm2, nullptr);

//   auto comm3 = std::make_unique<MockableFirmwareComm>("/dev/ttyACM0");
//   ASSERT_NE(comm3, nullptr);
// }

/**
 * @brief Test setCommandVelocities method
 */
// TEST_F(FirmwareCommTest, SetCommandVelocities)
// {
//   // Test setting valid velocities
//   std::vector<float> test_velocities = {1.5f, -2.3f};

//   // This should not throw any exceptions
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(test_velocities));

//   // Test with different velocity values
//   std::vector<float> zero_velocities = {0.0f, 0.0f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(zero_velocities));

//   std::vector<float> positive_velocities = {5.0f, 3.2f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(positive_velocities));

//   std::vector<float> negative_velocities = {-1.8f, -4.1f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(negative_velocities));
// }

/**
 * @brief Test setCommandVelocities with different vector sizes
 */
// TEST_F(FirmwareCommTest, SetCommandVelocitiesDifferentSizes)
// {
//   // Test with empty vector
//   std::vector<float> empty_velocities = {};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(empty_velocities));

//   // Test with single element
//   std::vector<float> single_velocity = {1.0f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(single_velocity));

//   // Test with more than 2 elements
//   std::vector<float> multi_velocities = {1.0f, 2.0f, 3.0f, 4.0f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(multi_velocities));
// }

/**
 * @brief Test getStatusVelocities method
 */
// TEST_F(FirmwareCommTest, GetStatusVelocities)
// {
//   // Get initial status velocities
//   std::vector<float> status_velocities = firmware_comm_->getStatusVelocities();

//   // Should return a vector (might be empty initially)
//   ASSERT_GE(status_velocities.size(), 0u);

//   // Test that method doesn't throw
//   ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
// }

/**
 * @brief Test getStatusPositionsStatus method
 */
// TEST_F(FirmwareCommTest, GetStatusPositions)
// {
//   // Get initial status positions
//   std::vector<float> status_positions = firmware_comm_->getStatusPositionsStatus();

//   // Should return a vector (might be empty initially)
//   ASSERT_GE(status_positions.size(), 0u);

//   // Test that method doesn't throw
//   ASSERT_NO_THROW(firmware_comm_->getStatusPositionsStatus());
// }

/**
 * @brief Test data consistency between set and get operations
 */
// TEST_F(FirmwareCommTest, DataConsistency)
// {
//   // Set some test velocities
//   std::vector<float> test_velocities = {2.5f, -1.8f};
//   firmware_comm_->setCommandVelocities(test_velocities);

//   // Verify that the object can handle multiple operations
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(test_velocities));
//   ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
//   ASSERT_NO_THROW(firmware_comm_->getStatusPositionsStatus());
// }

/**
 * @brief Test extreme values for velocities
 */
// TEST_F(FirmwareCommTest, ExtremeValues)
// {
//   // Test with very large positive values
//   std::vector<float> large_velocities = {1000.0f, 999.9f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(large_velocities));

//   // Test with very large negative values
//   std::vector<float> large_negative_velocities = {-1000.0f, -999.9f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(large_negative_velocities));

//   // Test with very small values
//   std::vector<float> small_velocities = {0.001f, -0.001f};
//   ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(small_velocities));
// }

/**
 * @brief Test interface compliance
 */
// TEST_F(FirmwareCommTest, InterfaceCompliance)
// {
//   // Test that FirmwareComm can be used through its interface
//   std::unique_ptr<IFirmwareComm> interface_ptr =
//     std::make_unique<MockableFirmwareComm>("/dev/null");

//   ASSERT_NE(interface_ptr, nullptr);

//   // Test interface methods
//   std::vector<float> test_velocities = {1.0f, 2.0f};
//   ASSERT_NO_THROW(interface_ptr->setCommandVelocities(test_velocities));
//   ASSERT_NO_THROW(interface_ptr->getStatusVelocities());
//   ASSERT_NO_THROW(interface_ptr->getStatusPositionsStatus());
// }

/**
 * @brief Test thread safety preparation
 *
 * This test verifies that the object can handle rapid successive calls,
 * which is important for thread safety.
 */
// TEST_F(FirmwareCommTest, RapidCalls)
// {
//   const int num_iterations = 100;

//   for (int i = 0; i < num_iterations; ++i) {
//     std::vector<float> velocities = {
//       static_cast<float>(i % 10),
//       static_cast<float>((i + 1) % 10)
//     };

//     ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(velocities));
//     ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
//     ASSERT_NO_THROW(firmware_comm_->getStatusPositionsStatus());
//   }
// }

/**
 * @brief Test memory management
 */
// TEST_F(FirmwareCommTest, MemoryManagement)
// {
//   // Test creating and destroying multiple instances
//   const int num_instances = 10;
//   std::vector<std::unique_ptr<MockableFirmwareComm>> instances;

//   // Create multiple instances
//   for (int i = 0; i < num_instances; ++i) {
//     auto instance = std::make_unique<MockableFirmwareComm>("/dev/null");
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

/**
 * @brief Performance test for basic operations
 */
// TEST_F(FirmwareCommTest, BasicPerformance)
// {
//   const int num_operations = 1000;
//   std::vector<float> test_velocities = {1.5f, -2.3f};

//   // Time the operations (basic check that they complete in reasonable time)
//   auto start = std::chrono::high_resolution_clock::now();

//   for (int i = 0; i < num_operations; ++i) {
//     firmware_comm_->setCommandVelocities(test_velocities);
//     firmware_comm_->getStatusVelocities();
//     firmware_comm_->getStatusPositionsStatus();
//   }

//   auto end = std::chrono::high_resolution_clock::now();
//   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

//   // Operations should complete within reasonable time (less than 1 second for 1000 operations)
//   ASSERT_LT(duration.count(), 1000);
// }
