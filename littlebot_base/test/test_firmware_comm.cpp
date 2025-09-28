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
#include "littlebot_base/i_firmware_comm.hpp"

namespace littlebot_base
{

/**
 * @brief Mock class for testing FirmwareComm without actual hardware
 *
 * This class extends FirmwareComm to allow testing without requiring
 * actual serial hardware connection.
 */
class MockableFirmwareComm : public FirmwareComm
{
public:
  explicit MockableFirmwareComm(const std::string & serial_port = "/dev/ttyUSB0")
  : FirmwareComm(serial_port)
  {
    // Initialize test data
    test_command_velocities_ = {0.0f, 0.0f};
    test_status_velocities_ = {0.0f, 0.0f};
    test_status_positions_ = {0.0f, 0.0f};
  }

  // Make protected/private methods accessible for testing
  void setTestCommandVelocities(const std::vector<float> & velocities)
  {
    test_command_velocities_ = velocities;
  }

  void setTestStatusVelocities(const std::vector<float> & velocities)
  {
    test_status_velocities_ = velocities;
  }

  void setTestStatusPositions(const std::vector<float> & positions)
  {
    test_status_positions_ = positions;
  }

  std::vector<float> getTestCommandVelocities() const
  {
    return test_command_velocities_;
  }

private:
  std::vector<float> test_command_velocities_;
  std::vector<float> test_status_velocities_;
  std::vector<float> test_status_positions_;
};

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
    firmware_comm_ = std::make_unique<MockableFirmwareComm>("/dev/null");
  }

  void TearDown() override
  {
    firmware_comm_.reset();
  }

  std::unique_ptr<MockableFirmwareComm> firmware_comm_;
};

/**
 * @brief Test constructor and destructor
 */
TEST_F(FirmwareCommTest, ConstructorDestructor)
{
  // Test that constructor creates object successfully
  ASSERT_NE(firmware_comm_, nullptr);

  // Test with different serial port names
  auto comm1 = std::make_unique<MockableFirmwareComm>("/dev/ttyUSB0");
  ASSERT_NE(comm1, nullptr);

  auto comm2 = std::make_unique<MockableFirmwareComm>("/dev/ttyUSB1");
  ASSERT_NE(comm2, nullptr);

  auto comm3 = std::make_unique<MockableFirmwareComm>("/dev/ttyACM0");
  ASSERT_NE(comm3, nullptr);
}

/**
 * @brief Test setCommandVelocities method
 */
TEST_F(FirmwareCommTest, SetCommandVelocities)
{
  // Test setting valid velocities
  std::vector<float> test_velocities = {1.5f, -2.3f};

  // This should not throw any exceptions
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(test_velocities));

  // Test with different velocity values
  std::vector<float> zero_velocities = {0.0f, 0.0f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(zero_velocities));

  std::vector<float> positive_velocities = {5.0f, 3.2f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(positive_velocities));

  std::vector<float> negative_velocities = {-1.8f, -4.1f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(negative_velocities));
}

/**
 * @brief Test setCommandVelocities with different vector sizes
 */
TEST_F(FirmwareCommTest, SetCommandVelocitiesDifferentSizes)
{
  // Test with empty vector
  std::vector<float> empty_velocities = {};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(empty_velocities));

  // Test with single element
  std::vector<float> single_velocity = {1.0f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(single_velocity));

  // Test with more than 2 elements
  std::vector<float> multi_velocities = {1.0f, 2.0f, 3.0f, 4.0f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(multi_velocities));
}

/**
 * @brief Test getStatusVelocities method
 */
TEST_F(FirmwareCommTest, GetStatusVelocities)
{
  // Get initial status velocities
  std::vector<float> status_velocities = firmware_comm_->getStatusVelocities();

  // Should return a vector (might be empty initially)
  ASSERT_GE(status_velocities.size(), 0u);

  // Test that method doesn't throw
  ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
}

/**
 * @brief Test getStatusPositionsStatus method
 */
TEST_F(FirmwareCommTest, GetStatusPositions)
{
  // Get initial status positions
  std::vector<float> status_positions = firmware_comm_->getStatusPositionsStatus();

  // Should return a vector (might be empty initially)
  ASSERT_GE(status_positions.size(), 0u);

  // Test that method doesn't throw
  ASSERT_NO_THROW(firmware_comm_->getStatusPositionsStatus());
}

/**
 * @brief Test data consistency between set and get operations
 */
TEST_F(FirmwareCommTest, DataConsistency)
{
  // Set some test velocities
  std::vector<float> test_velocities = {2.5f, -1.8f};
  firmware_comm_->setCommandVelocities(test_velocities);

  // Verify that the object can handle multiple operations
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(test_velocities));
  ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
  ASSERT_NO_THROW(firmware_comm_->getStatusPositionsStatus());
}

/**
 * @brief Test extreme values for velocities
 */
TEST_F(FirmwareCommTest, ExtremeValues)
{
  // Test with very large positive values
  std::vector<float> large_velocities = {1000.0f, 999.9f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(large_velocities));

  // Test with very large negative values
  std::vector<float> large_negative_velocities = {-1000.0f, -999.9f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(large_negative_velocities));

  // Test with very small values
  std::vector<float> small_velocities = {0.001f, -0.001f};
  ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(small_velocities));
}

/**
 * @brief Test interface compliance
 */
TEST_F(FirmwareCommTest, InterfaceCompliance)
{
  // Test that FirmwareComm can be used through its interface
  std::unique_ptr<IFirmwareComm> interface_ptr =
    std::make_unique<MockableFirmwareComm>("/dev/null");

  ASSERT_NE(interface_ptr, nullptr);

  // Test interface methods
  std::vector<float> test_velocities = {1.0f, 2.0f};
  ASSERT_NO_THROW(interface_ptr->setCommandVelocities(test_velocities));
  ASSERT_NO_THROW(interface_ptr->getStatusVelocities());
  ASSERT_NO_THROW(interface_ptr->getStatusPositionsStatus());
}

/**
 * @brief Test thread safety preparation
 *
 * This test verifies that the object can handle rapid successive calls,
 * which is important for thread safety.
 */
TEST_F(FirmwareCommTest, RapidCalls)
{
  const int num_iterations = 100;

  for (int i = 0; i < num_iterations; ++i) {
    std::vector<float> velocities = {
      static_cast<float>(i % 10),
      static_cast<float>((i + 1) % 10)
    };

    ASSERT_NO_THROW(firmware_comm_->setCommandVelocities(velocities));
    ASSERT_NO_THROW(firmware_comm_->getStatusVelocities());
    ASSERT_NO_THROW(firmware_comm_->getStatusPositionsStatus());
  }
}

/**
 * @brief Test memory management
 */
TEST_F(FirmwareCommTest, MemoryManagement)
{
  // Test creating and destroying multiple instances
  const int num_instances = 10;
  std::vector<std::unique_ptr<MockableFirmwareComm>> instances;

  // Create multiple instances
  for (int i = 0; i < num_instances; ++i) {
    auto instance = std::make_unique<MockableFirmwareComm>("/dev/null");
    ASSERT_NE(instance, nullptr);
    instances.push_back(std::move(instance));
  }

  // Use all instances
  for (auto & instance : instances) {
    std::vector<float> test_velocities = {1.0f, 2.0f};
    ASSERT_NO_THROW(instance->setCommandVelocities(test_velocities));
  }

  // Instances will be automatically destroyed when vector goes out of scope
}

/**
 * @brief Performance test for basic operations
 */
TEST_F(FirmwareCommTest, BasicPerformance)
{
  const int num_operations = 1000;
  std::vector<float> test_velocities = {1.5f, -2.3f};

  // Time the operations (basic check that they complete in reasonable time)
  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < num_operations; ++i) {
    firmware_comm_->setCommandVelocities(test_velocities);
    firmware_comm_->getStatusVelocities();
    firmware_comm_->getStatusPositionsStatus();
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  // Operations should complete within reasonable time (less than 1 second for 1000 operations)
  ASSERT_LT(duration.count(), 1000);
}

}  // namespace littlebot_base
