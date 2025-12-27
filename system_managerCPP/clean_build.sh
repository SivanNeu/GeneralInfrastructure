#!/bin/bash
# Clean build script
echo "Cleaning build directory..."
rm -rf build/*
rm -rf build/.* 2>/dev/null
echo "Build directory cleaned. Run 'mkdir -p build && cd build && cmake .. && make' to rebuild."
