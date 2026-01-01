#!/bin/bash

# Build script for SystemManagerCPP

set -e  # Exit on error

BUILD_TYPE="${1:-Release}"

# Validate build type
if [[ ! "$BUILD_TYPE" =~ ^(Debug|Release|RelWithDebInfo|MinSizeRel)$ ]]; then
    echo "Error: Invalid build type '$BUILD_TYPE'"
    echo "Valid options: Debug, Release, RelWithDebInfo, MinSizeRel"
    exit 1
fi

echo "Building SystemManagerCPP..."
echo "Build type: $BUILD_TYPE"
echo "Note: RLPolicyClean uses JSON-based inference (no LibTorch required)"

# Create build directory
mkdir -p build
cd build

# Configure CMake
CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"

cmake .. $CMAKE_ARGS

# Build
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "Build complete! Library is in: build/lib/"
echo "To install: make install"

