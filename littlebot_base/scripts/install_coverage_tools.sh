#!/bin/bash

# Script to install test coverage tools for littlebot_base

echo "Installing test coverage tools..."

# Update package list
sudo apt update

# Install lcov and gcovr
sudo apt install -y lcov gcovr

# Check if tools are installed correctly
echo "Checking installation..."

if command -v lcov &> /dev/null; then
    echo "✓ lcov installed: $(lcov --version | head -n1)"
else
    echo "✗ lcov not found"
fi

if command -v genhtml &> /dev/null; then
    echo "✓ genhtml installed (part of lcov)"
else
    echo "✗ genhtml not found"
fi

if command -v gcovr &> /dev/null; then
    echo "✓ gcovr installed: $(gcovr --version)"
else
    echo "✗ gcovr not found"
fi

echo "Coverage tools installation complete!"
echo ""
echo "Usage:"
echo "  1. Build with coverage: colcon build --cmake-args -DENABLE_COVERAGE=ON"
echo "  2. Run tests with coverage: colcon test --packages-select littlebot_base"
echo "  3. Generate coverage report:"
echo "     - make coverage (default - prefers lcov)"
echo "     - make coverage_lcov (lcov HTML report)"
echo "     - make coverage_gcovr (gcovr HTML report)"
echo "     - make coverage_xml (gcovr XML report)"
echo "     - make coverage_summary (gcovr console summary)"
echo "  4. Open coverage report: xdg-open build/littlebot_base/coverage/index.html"