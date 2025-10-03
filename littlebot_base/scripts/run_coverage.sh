#!/bin/bash

# Script to run test coverage for littlebot_base

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BUILD_DIR="$WORKSPACE_DIR/build/littlebot_base"

echo "Test Coverage Script for littlebot_base"
echo "======================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Build dir: $BUILD_DIR"
echo ""

# Check if coverage tools are available
if ! command -v lcov &> /dev/null && ! command -v gcovr &> /dev/null; then
    echo "Error: Neither lcov nor gcovr found!"
    echo "Please install coverage tools:"
    echo "  sudo apt install lcov gcovr"
    echo "Or run: ./scripts/install_coverage_tools.sh"
    exit 1
fi

# Change to workspace directory
cd "$WORKSPACE_DIR"

echo "Step 1: Clean previous build..."
rm -rf build/littlebot_base install/littlebot_base log/littlebot_base

echo "Step 2: Build with coverage enabled..."
colcon build --packages-select littlebot_base --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug

echo "Step 3: Run tests..."
colcon test --packages-select littlebot_base --event-handlers console_direct+

echo "Step 4: Generate coverage report..."
cd "$BUILD_DIR"

if command -v lcov &> /dev/null && command -v genhtml &> /dev/null; then
    echo "Using lcov for coverage reporting..."
    make coverage_lcov
    REPORT_FILE="$BUILD_DIR/coverage/index.html"
elif command -v gcovr &> /dev/null; then
    echo "Using gcovr for coverage reporting..."
    make coverage_gcovr
    REPORT_FILE="$BUILD_DIR/coverage/index.html"
else
    echo "Error: No suitable coverage tool found!"
    exit 1
fi

echo ""
echo "Coverage report generated successfully!"
echo "HTML Report: $REPORT_FILE"

if command -v xdg-open &> /dev/null; then
    echo "Opening coverage report in browser..."
    xdg-open "$REPORT_FILE"
fi

echo ""
echo "Coverage Summary:"
if command -v lcov &> /dev/null && [ -f coverage_filtered.info ]; then
    lcov --summary coverage_filtered.info 2>/dev/null || echo "Run 'lcov --summary coverage_filtered.info' for detailed summary"
elif command -v gcovr &> /dev/null; then
    echo "Run 'make coverage_summary' for detailed gcovr summary"
fi