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

echo "Unit tests completed!"