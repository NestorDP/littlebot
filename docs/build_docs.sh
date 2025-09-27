#!/bin/bash

# LittleBot Documentation Build Script
# This script builds the complete documentation for the LittleBot project

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

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCS_DIR="$SCRIPT_DIR"

print_status "Building LittleBot documentation..."
print_status "Documentation directory: $DOCS_DIR"

# Change to docs directory
cd "$DOCS_DIR"

# Check if virtual environment exists
if [ ! -d ".venv" ]; then
    print_status "Creating Python virtual environment..."
    python3 -m venv .venv
fi

# Activate virtual environment
print_status "Activating virtual environment..."
source .venv/bin/activate

# Install dependencies
print_status "Installing Python dependencies..."
pip install --upgrade pip
pip install sphinx sphinx-rtd-theme sphinx-autoapi breathe myst-parser

# Check if Doxygen is installed
if ! command -v doxygen &> /dev/null; then
    print_warning "Doxygen not found. Installing..."
    sudo apt update
    sudo apt install -y doxygen graphviz
fi

# Clean previous builds
print_status "Cleaning previous builds..."
rm -rf build/
rm -rf doxygen/

# Generate C++ API documentation with Doxygen
print_status "Generating C++ API documentation with Doxygen..."
if [ -f "Doxyfile" ]; then
    doxygen Doxyfile
    print_success "Doxygen documentation generated"
else
    print_warning "Doxyfile not found, skipping C++ API generation"
fi

# Build Sphinx documentation
print_status "Building Sphinx documentation..."
make html

# Check if build was successful
if [ -f "build/html/index.html" ]; then
    print_success "Documentation built successfully!"
    print_status "Documentation location: $DOCS_DIR/build/html/"
    print_status "Open $DOCS_DIR/build/html/index.html in your browser"
    
    # Option to serve the documentation
    read -p "Do you want to serve the documentation locally? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_status "Starting local server on http://localhost:8000"
        print_status "Press Ctrl+C to stop the server"
        cd build/html
        python3 -m http.server 8000
    fi
else
    print_error "Documentation build failed!"
    exit 1
fi