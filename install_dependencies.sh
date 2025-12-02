#!/usr/bin/env bash
set -e

# Ensure vcs and rosdep are installed
if ! command -v vcs &> /dev/null; then
  echo "[INFO] Installing python3-vcstool..."
  sudo apt-get update && sudo apt-get install -y python3-vcstool
fi

if ! command -v rosdep &> /dev/null; then
  echo "[INFO] Installing python3-rosdep..."
  sudo apt-get update && sudo apt-get install -y python3-rosdep
fi
 
# Import external sources (protobuf, cppserial, etc.)
if [ -f src/littlebot/littlebot.repos ]; then
  echo "[INFO] Importing external sources with vcs..."
  vcs import src < src/littlebot/littlebot.repos
else
  echo "[WARN] littlebot.repos not found. Skipping vcs import."
fi


# Initialize and update rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "[INFO] Initializing rosdep..."
  sudo rosdep init || true
fi

echo "[INFO] Updating rosdep..."
rosdep update

# Install all dependencies (ROS, system, and 3rd-party)
echo "[INFO] Installing dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y


# Install any additional system dependencies not covered by rosdep
echo "[INFO] Installing system dependencies..."
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libqwt-qt5-6 \
    libqwt-qt5-dev

echo "[INFO] Instaling gtest..."
sudo apt-get install -y libgtest-dev
# Build and install gtest
cd /usr/src/gtest
sudo mkdir -p build && cd build
sudo cmake ..
sudo make
sudo cp lib/*.a /usr/lib/

cd ~/littlebot_ws  # Return to littlebot directory

# Optionally build/install protobuf and cppserial if not available as system packages
# (Assumes their repos were imported to src/)

PROTOBUF_DIR="src/protobuf"
CPPSERIAL_DIR="src/cppserial"

if [ -d "$PROTOBUF_DIR" ]; then
  echo "[INFO] Building and installing protobuf from source..."
  cd "$PROTOBUF_DIR"
  git submodule update --init --recursive
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  sudo ldconfig
  cd ../../..
fi

if [ -d "$CPPSERIAL_DIR" ]; then
  echo "[INFO] Building and installing cppserial from source..."
  cd "$CPPSERIAL_DIR"
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  sudo ldconfig
  cd ../../..
fi

echo "[INFO] All dependencies installed. You can now build the workspace with colcon build."
ls