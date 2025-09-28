# LittleBot Dependencies Installation

This document describes how to automatically install the external dependencies for the LittleBot project, particularly the custom `libserial` library.

## Quick Start

The easiest way to install all dependencies is to run the provided installation script:

```bash
cd /path/to/littlebot_ws
./install_dependencies.sh
```

## Installation Options

### Option 1: Automated Installation Script (Recommended)

Run the comprehensive installation script that handles everything:

```bash
# Make sure you're in the workspace root
cd ~/littlebot_ws

# Run the installation script
./install_dependencies.sh

# Build the workspace
colcon build
```

This script will:
- Install system dependencies
- Clone and build libserial from GitHub
- Install ROS2 dependencies via rosdep
- Verify the installation

### Option 2: Using vcstool (For Developers)

If you're setting up a development environment, use vcstool to manage all repositories:

```bash
# Import all repositories including dependencies
cd ~/littlebot_ws
vcs import src < src/littlebot.repos

# Install system dependencies
sudo apt update
sudo apt install -y git cmake build-essential pkg-config libudev-dev

# Build libserial first
cd src/libserial
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

# Go back to workspace and build everything
cd ~/littlebot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Option 3: Manual Installation

If you prefer to install dependencies manually:

```bash
# Install system dependencies
sudo apt update
sudo apt install -y git cmake build-essential pkg-config libudev-dev protobuf-compiler libprotobuf-dev

# Clone and build libserial
git clone https://github.com/NestorDP/libserial.git /tmp/libserial
cd /tmp/libserial
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig

# Clean up
rm -rf /tmp/libserial

# Install ROS2 dependencies
cd ~/littlebot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
```

### Option 4: Docker-based Installation

For containerized development:

```dockerfile
# Add to your Dockerfile
RUN apt-get update && apt-get install -y \
    git cmake build-essential pkg-config libudev-dev protobuf-compiler libprotobuf-dev

# Install libserial
RUN git clone https://github.com/NestorDP/libserial.git /tmp/libserial && \
    cd /tmp/libserial && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/libserial
```

## Verification

After installation, verify that dependencies are properly installed:

```bash
# Check libserial
pkg-config --exists libserial && echo "libserial found via pkg-config"
ls -la /usr/local/lib/libserial* && echo "libserial library files found"
ls -la /usr/local/include/libserial/ && echo "libserial headers found"

# Check protobuf
pkg-config --exists protobuf && echo "protobuf found via pkg-config"
protoc --version && echo "protobuf compiler is available"
dpkg -l | grep -E "protobuf|libprotobuf" && echo "protobuf packages installed"

# Test the build
cd ~/littlebot_ws
colcon build --packages-select littlebot_base
```

## Troubleshooting

### libserial Not Found

If CMake can't find libserial:

```bash
# Check installation paths
echo $PKG_CONFIG_PATH
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

# Update linker cache
sudo ldconfig

# Verify installation
pkg-config --modversion libserial
```

### Permission Issues

If you encounter permission issues:

```bash
# Make sure you're not running as root
whoami  # Should not return 'root'

# Check file permissions
ls -la /usr/local/lib/libserial*
ls -la /usr/local/include/libserial/
```

### Build Errors

If the build fails:

```bash
# Clean and rebuild
cd ~/littlebot_ws
rm -rf build install log
colcon build --packages-select littlebot_base --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

## Integration with CI/CD

For continuous integration, add this to your workflow:

```yaml
# GitHub Actions example
- name: Install dependencies
  run: |
    cd $GITHUB_WORKSPACE
    ./install_dependencies.sh

- name: Build workspace
  run: |
    cd $GITHUB_WORKSPACE
    colcon build --packages-select littlebot_base
```

## External Dependencies

### Custom libserial Repository

The libserial dependency is maintained at: https://github.com/NestorDP/libserial

This is a fork/custom version of the original libserial library with modifications specific to the LittleBot project.

### Protocol Buffers

Protocol Buffers (protobuf) is used for message serialization in firmware communication. The dependency uses the official Google repository: https://github.com/protocolbuffers/protobuf

**System Installation (Recommended):**
```bash
sudo apt install protobuf-compiler libprotobuf-dev
```

**From Source (Development):**
If you need the latest version or want to build from source:
```bash
# Using vcstool (includes protobuf in workspace)
vcs import src < src/littlebot.repos

# Manual installation
git clone https://github.com/protocolbuffers/protobuf.git /tmp/protobuf
cd /tmp/protobuf
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF
cmake --build build --parallel $(nproc)
sudo cmake --install build
```

## Support

If you encounter issues with dependency installation:

1. Check the [Troubleshooting](#troubleshooting) section above
2. Verify your system meets the requirements
3. Open an issue in the LittleBot repository with:
   - Your OS version
   - Error messages
   - Steps you've tried