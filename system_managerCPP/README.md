# SystemManagerCPP

C++ implementation of the System Manager for quadcopter control.

## Dependencies

### Required
- **CMake** (version 3.15 or higher)
- **C++17** compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- **Eigen3** - Linear algebra library
  ```bash
  # Ubuntu/Debian
  sudo apt-get install libeigen3-dev
  
  # macOS
  brew install eigen
  ```
- **ZeroMQ** and **cppzmq** - Message passing library
  ```bash
  # Ubuntu/Debian
  sudo apt-get install libzmq3-dev libcppzmq-dev
  
  # macOS
  brew install zeromq cppzmq
  ```

### Optional
- **LibTorch** - PyTorch C++ API (required for RLPolicyClean)
  - Download from: https://pytorch.org/get-started/locally/
  - **Default location**: `/opt/libtorch` (automatically searched)
  - Alternative: Extract to any location and set `LIBTORCH_PATH` or `CMAKE_PREFIX_PATH`

## Building

### Standard Build

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)  # or make -j4 on macOS
```

### Build with LibTorch

LibTorch is automatically searched in `/opt/libtorch` by default. If installed elsewhere:

```bash
mkdir build
cd build
# Option 1: Use LIBTORCH_PATH variable
cmake -DLIBTORCH_PATH=/path/to/libtorch ..

# Option 2: Use CMAKE_PREFIX_PATH
cmake -DCMAKE_PREFIX_PATH=/path/to/libtorch ..

# Option 3: If in /opt/libtorch, no extra flags needed
cmake ..
make -j$(nproc)
```

### Build Debug Version

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

### Build without LibTorch (disable RLPolicyClean)

```bash
mkdir build
cd build
cmake -DUSE_LIBTORCH=OFF ..
make -j$(nproc)
```

### Build Options

- `CMAKE_BUILD_TYPE`: `Debug`, `Release`, `RelWithDebInfo`, or `MinSizeRel` (default: Release)
- `USE_LIBTORCH`: Enable/disable LibTorch support (default: ON)
- `LIBTORCH_PATH`: Path to LibTorch installation (default: `/opt/libtorch`)
- `USE_SANITIZERS`: Enable address/undefined sanitizers in Debug mode (default: ON)

### Build Type Details

- **Debug**: Full debug symbols (`-g3`), no optimization (`-O0`), debug macros enabled, optional sanitizers
- **Release**: Optimized (`-O3`), no debug symbols, `NDEBUG` defined
- **RelWithDebInfo**: Optimized (`-O2`) with debug symbols (`-g`), `NDEBUG` defined
- **MinSizeRel**: Optimized for size (`-Os`), `NDEBUG` defined

## Project Structure

```
system_managerCPP/
├── CMakeLists.txt
├── Include/
│   ├── Control.h
│   ├── SystemManager.h
│   ├── VelocityPIDController.h
│   ├── VelocityRLController.h
│   ├── RLPolicyClean.h
│   └── utils/          # Utility headers
├── Src/
│   ├── Control.cpp
│   ├── SystemManager.cpp
│   ├── VelocityPIDController.cpp
│   ├── VelocityRLController.cpp
│   ├── RLPolicyClean.cpp
│   └── utils/          # Utility implementations
└── README.md
```

## Usage

The project builds a static library `libSystemManagerLib.a` that can be linked to your application.

Example:
```cmake
target_link_libraries(your_target SystemManagerLib)
```

## Notes

- The RLPolicyClean class requires LibTorch. If LibTorch is not available, the code will compile but RL inference will not work.
- ZMQ communication uses ports defined in `SystemManager.h` (default: 7790 for flight data, 7793 for commands).

