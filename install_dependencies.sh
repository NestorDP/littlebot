#!/usr/bin/env bash
set -e

# 1. Ensure vcs and rosdep are installed
if ! command -v vcs &> /dev/null; then
  echo "[INFO] Installing python3-vcstool..."
  sudo apt-get update && sudo apt-get install -y python3-vcstool
fi

if ! command -v rosdep &> /dev/null; then
  echo "[INFO] Installing python3-rosdep..."
  sudo apt-get update && sudo apt-get install -y python3-rosdep
fi

# 2. Import external sources (protobuf, cppserial, etc.)
if [ -f littlebot.repos ]; then
  echo "[INFO] Importing external sources with vcs..."
  vcs import src < littlebot.repos
else
  echo "[WARN] littlebot.repos not found. Skipping vcs import."
fi

# 3. Initialize and update rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "[INFO] Initializing rosdep..."
  sudo rosdep init || true
fi

echo "[INFO] Updating rosdep..."
rosdep update

# 4. Install all dependencies (ROS, system, and 3rd-party)
echo "[INFO] Installing dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y

# 5. Optionally build/install protobuf and cppserial if not available as system packages
# (Assumes their repos were imported to src/)

PROTOBUF_DIR="src/protobuf"
CPPSERIAL_DIR="src/cppserial"

if [ -d "$PROTOBUF_DIR" ]; then
  echo "[INFO] Building and installing protobuf from source..."
  cd "$PROTOBUF_DIR"
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  sudo ldconfig
  cd ../../../..
fi

if [ -d "$CPPSERIAL_DIR" ]; then
  echo "[INFO] Building and installing cppserial from source..."
  cd "$CPPSERIAL_DIR"
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  sudo ldconfig
  cd ../../../..
fi

echo "[INFO] All dependencies installed. You can now build the workspace with colcon build."
