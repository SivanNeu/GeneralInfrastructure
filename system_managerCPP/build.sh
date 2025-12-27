#!/bin/bash

# Build script for SystemManagerCPP

set -e  # Exit on error

BUILD_TYPE="${1:-Release}"
USE_LIBTORCH="${2:-ON}"
LIBTORCH_PATH="${3:-}"

# Validate build type
if [[ ! "$BUILD_TYPE" =~ ^(Debug|Release|RelWithDebInfo|MinSizeRel)$ ]]; then
    echo "Error: Invalid build type '$BUILD_TYPE'"
    echo "Valid options: Debug, Release, RelWithDebInfo, MinSizeRel"
    exit 1
fi

echo "Building SystemManagerCPP..."
echo "Build type: $BUILD_TYPE"
echo "Use LibTorch: $USE_LIBTORCH"

# Create build directory
mkdir -p build
cd build

# Configure CMake
CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE -DUSE_LIBTORCH=$USE_LIBTORCH"

if [ ! -z "$LIBTORCH_PATH" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_PREFIX_PATH=$LIBTORCH_PATH"
fi

cmake .. $CMAKE_ARGS

# Build
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "Build complete! Library is in: build/lib/"
echo "To install: make install"

