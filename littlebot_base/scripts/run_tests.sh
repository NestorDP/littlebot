#!/bin/bash

# Script to run littlebot_base unit tests
# This script builds and runs the unit tests for the FirmwareComm class

set -e

echo "Building and running littlebot_base unit tests..."

# Go to workspace root
cd "$(dirname "$0")/../../../.."

# Build the package with tests enabled
echo "Building package with tests..."
colcon build --packages-select littlebot_base --cmake-args -DBUILD_TESTING=ON

# Run the tests
echo "Running unit tests..."
colcon test --packages-select littlebot_base

# Show test results
echo "Test results:"
colcon test-result --verbose

# List available tests
echo "Available tests:"
./build/littlebot_base/littlebot_base_unit_tests --gtest_list_tests

# Run all tests
echo "Running all tests..."
./build/littlebot_base/littlebot_base_unit_tests

echo "Unit tests completed!"