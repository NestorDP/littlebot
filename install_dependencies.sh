#!/bin/bash

# LittleBot Dependencies Installation Script
# This script automatically installs all required dependencies for LittleBot

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_status "Installing LittleBot dependencies..."

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root (don't use sudo)"
   exit 1
fi

# Update package list
print_status "Updating package list..."
sudo apt update

# Install system dependencies
print_status "Installing system dependencies..."
sudo apt install -y \
    git \
    cmake \
    build-essential \
    pkg-config \
    libudev-dev \
    protobuf-compiler \
    libprotobuf-dev

# Install libserial from GitHub
LIBSERIAL_DIR="/tmp/libserial_build"
LIBSERIAL_INSTALL_FLAG="/usr/local/lib/libserial.so"

if [ ! -f "$LIBSERIAL_INSTALL_FLAG" ]; then
    print_status "Installing libserial from GitHub repository..."
    
    # Clean up any previous build
    if [ -d "$LIBSERIAL_DIR" ]; then
        rm -rf "$LIBSERIAL_DIR"
    fi
    
    # Clone and build libserial
    git clone https://github.com/NestorDP/libserial.git "$LIBSERIAL_DIR"
    cd "$LIBSERIAL_DIR"
    
    print_status "Configuring libserial build..."
    mkdir -p build && cd build
    
    # Configure with CMake
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_SHARED_LIBS=ON
    
    print_status "Building libserial..."
    make -j$(nproc)
    
    print_status "Installing libserial..."
    sudo make install
    
    # Update linker cache
    sudo ldconfig
    
    # Clean up build directory
    cd /
    rm -rf "$LIBSERIAL_DIR"
    
    print_success "libserial installed successfully"
else
    print_success "libserial is already installed"
fi

# Verify installations
print_status "Verifying dependencies installation..."

# Verify libserial
if pkg-config --exists libserial; then
    LIBSERIAL_VERSION=$(pkg-config --modversion libserial)
    print_success "libserial version $LIBSERIAL_VERSION is installed"
elif [ -f "/usr/local/lib/libserial.so" ]; then
    print_success "libserial library found at /usr/local/lib/libserial.so"
else
    print_error "libserial installation verification failed"
    exit 1
fi

# Verify protobuf
if pkg-config --exists protobuf; then
    PROTOBUF_VERSION=$(pkg-config --modversion protobuf)
    print_success "protobuf version $PROTOBUF_VERSION is installed"
elif command -v protoc >/dev/null 2>&1; then
    PROTOBUF_VERSION=$(protoc --version | cut -d' ' -f2)
    print_success "protobuf compiler version $PROTOBUF_VERSION is installed"
else
    print_error "protobuf installation verification failed"
    exit 1
fi

# Install ROS2 dependencies using rosdep
print_status "Installing ROS2 dependencies..."
cd "$(dirname "$0")/.."  # Go to workspace root

# Check if rosdep is initialized
if [ ! -d "/etc/ros/rosdep" ]; then
    print_status "Initializing rosdep..."
    sudo rosdep init
fi

print_status "Updating rosdep database..."
rosdep update

# Install dependencies for all packages
print_status "Installing ROS2 package dependencies..."
rosdep install --from-paths src --ignore-src -r -y

print_success "All dependencies installed successfully!"
print_status "Dependencies installed:"
print_status "  - libserial (custom build from GitHub)"
print_status "  - Protocol Buffers (system package)"
print_status "  - ROS2 package dependencies"
print_status ""
print_status "You can now build the workspace with: colcon build"