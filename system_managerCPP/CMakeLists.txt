cmake_minimum_required(VERSION 3.15)
project(SystemManagerCPP VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type - default to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Set possible build types
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Include)
set(INCLUDE_UTILS_DIR ${CMAKE_SOURCE_DIR}/Include/utils)

include_directories(
    ${INCLUDE_DIR}
    ${INCLUDE_UTILS_DIR}
)

# Source files - main directory
set(MAIN_SOURCES
    Src/Control.cpp
    Src/IntegralUtils.cpp
    Src/MatrixUtils.cpp
    Src/RLPolicyClean.cpp
    Src/SystemManager.cpp
    Src/VelocityPIDController.cpp
    Src/VelocityRLController.cpp
)

# Source files - utils directory
set(UTILS_SOURCES
    Src/utils/Altitude.cpp
    Src/utils/CameraType.cpp
    Src/utils/Euler.cpp
    Src/utils/Feature.cpp
    Src/utils/FeatureList.cpp
    Src/utils/FlightData.cpp
    Src/utils/GeneralFuncs.cpp
    Src/utils/Imu.cpp
    Src/utils/LLA.cpp
    Src/utils/Logger.cpp
    Src/utils/LowPassFilter.cpp
    Src/utils/NED.cpp
    Src/utils/Pixel.cpp
    Src/utils/Quaternion.cpp
    Src/utils/RateCmd.cpp
    Src/utils/TimeUtils.cpp
    Src/utils/Tuning.cpp
    Src/utils/TuningState.cpp
    Src/utils/TuningSubstate.cpp
    Src/utils/Utils.cpp
    Src/utils/VirtualTarget.cpp
)

# Combine all sources
set(ALL_SOURCES ${MAIN_SOURCES} ${UTILS_SOURCES})

# Find required packages
# Eigen3
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen3.")
endif()

# ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Find cppzmq header
find_path(CPPZMQ_INCLUDE_DIR 
    NAMES zmq.hpp
    PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
    ${CMAKE_SOURCE_DIR}/third_party/cppzmq
)

if(CPPZMQ_INCLUDE_DIR)
    message(STATUS "cppzmq found: ${CPPZMQ_INCLUDE_DIR}")
    include_directories(${CPPZMQ_INCLUDE_DIR})
else()
    message(FATAL_ERROR "cppzmq not found. Please install cppzmq-dev or libcppzmq-dev")
endif()

# Build options
option(USE_LIBTORCH "Build with LibTorch support for RLPolicyClean" ON)
option(USE_SANITIZERS "Enable address and undefined behavior sanitizers in Debug mode" ON)
option(LIBTORCH_PATH "Path to LibTorch installation" "/opt/libtorch")

if(USE_LIBTORCH)
    # First try the default path /opt/libtorch
    if(EXISTS "${LIBTORCH_PATH}/share/cmake/Torch/TorchConfig.cmake")
        set(CMAKE_PREFIX_PATH "${LIBTORCH_PATH}" ${CMAKE_PREFIX_PATH})
        message(STATUS "Using LibTorch from default path: ${LIBTORCH_PATH}")
    endif()
    
    # Try to find Torch package
    find_package(Torch QUIET)
    
    # If not found, try common alternative locations
    if(NOT Torch_FOUND)
        set(LIBTORCH_SEARCH_PATHS
            "/opt/libtorch"
            "/usr/local/libtorch"
            "$ENV{HOME}/libtorch"
            "${CMAKE_SOURCE_DIR}/third_party/libtorch"
            "${CMAKE_SOURCE_DIR}/../libtorch"
        )
        
        foreach(SEARCH_PATH ${LIBTORCH_SEARCH_PATHS})
            if(EXISTS "${SEARCH_PATH}/share/cmake/Torch/TorchConfig.cmake")
                set(CMAKE_PREFIX_PATH "${SEARCH_PATH}" ${CMAKE_PREFIX_PATH})
                message(STATUS "Trying LibTorch from: ${SEARCH_PATH}")
                find_package(Torch QUIET)
                if(Torch_FOUND)
                    break()
                endif()
            endif()
        endforeach()
    endif()
    
    if(Torch_FOUND)
        message(STATUS "LibTorch found: ${TORCH_INSTALL_PREFIX}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
    else()
        message(WARNING "LibTorch not found. RLPolicyClean will not work properly.")
        message(WARNING "Searched in:")
        message(WARNING "  - ${LIBTORCH_PATH} (default)")
        foreach(SEARCH_PATH ${LIBTORCH_SEARCH_PATHS})
            message(WARNING "  - ${SEARCH_PATH}")
        endforeach()
        message(WARNING "To use LibTorch, download from https://pytorch.org/get-started/locally/")
        message(WARNING "and either:")
        message(WARNING "  1. Place it in /opt/libtorch, or")
        message(WARNING "  2. Set CMAKE_PREFIX_PATH to the LibTorch installation directory, or")
        message(WARNING "  3. Set LIBTORCH_PATH variable: cmake -DLIBTORCH_PATH=/path/to/libtorch ..")
        set(USE_LIBTORCH OFF)
    endif()
endif()

# Create library
add_library(SystemManagerLib STATIC ${ALL_SOURCES})

# Compiler flags - set on target (must be after target creation)
# Common flags for all build types
target_compile_options(SystemManagerLib PRIVATE
    -Wall -Wextra
)

# Build-type specific flags using generator expressions
target_compile_options(SystemManagerLib PRIVATE
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:RelWithDebInfo>:-O2>
    $<$<CONFIG:RelWithDebInfo>:-g>
    $<$<CONFIG:MinSizeRel>:-Os>
)

# Define macros separately to avoid redefinition warnings
# CMake may already define NDEBUG, so we use compile definitions
target_compile_definitions(SystemManagerLib PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
    # Note: NDEBUG is typically defined by CMake for Release builds
    # We don't explicitly define it to avoid redefinition warnings
)

# Link libraries
target_link_libraries(SystemManagerLib
    PUBLIC
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
)

# Include directories for the library
target_include_directories(SystemManagerLib
    PUBLIC
    ${INCLUDE_DIR}
    ${INCLUDE_UTILS_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${ZMQ_INCLUDE_DIRS}
)

# Link LibTorch if available
if(USE_LIBTORCH AND Torch_FOUND)
    target_link_libraries(SystemManagerLib PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(SystemManagerLib PUBLIC USE_LIBTORCH)
endif()

# Compiler-specific options
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(SystemManagerLib PRIVATE -Wno-unused-parameter)
    
    # Additional debug flags for GCC/Clang
    target_compile_options(SystemManagerLib PRIVATE 
        $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    )
    
    # Sanitizers (optional, can be disabled for faster compilation)
    if(USE_SANITIZERS)
        target_compile_options(SystemManagerLib PRIVATE 
            $<$<CONFIG:Debug>:-fsanitize=address>
            $<$<CONFIG:Debug>:-fsanitize=undefined>
        )
        target_link_options(SystemManagerLib PRIVATE 
            $<$<CONFIG:Debug>:-fsanitize=address>
            $<$<CONFIG:Debug>:-fsanitize=undefined>
        )
    endif()
endif()

# MSVC-specific options
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(SystemManagerLib PRIVATE 
        $<$<CONFIG:Debug>:/W4 /Zi /Od>
        $<$<CONFIG:Release>:/W4 /O2>
    )
endif()

# Example executable (optional - uncomment if you want to build an executable)
# Create executable
add_executable(SystemManagerMain Src/main.cpp)

# Link the executable with the library
target_link_libraries(SystemManagerMain
    PRIVATE
    SystemManagerLib
)

# Include directories for the executable
target_include_directories(SystemManagerMain
    PRIVATE
    ${INCLUDE_DIR}
    ${INCLUDE_UTILS_DIR}
)
# target_link_libraries(SystemManagerMain SystemManagerLib)

# Installation (optional)
install(TARGETS SystemManagerLib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${INCLUDE_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# Print configuration summary
message(STATUS "")
message(STATUS "========== SystemManagerCPP Configuration ==========")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "  Debug flags: -g3 -O0 -DDEBUG")
    if(USE_SANITIZERS AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
        message(STATUS "  Sanitizers: Address, Undefined (enabled)")
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "  Release flags: -O3 -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message(STATUS "  RelWithDebInfo flags: -O2 -g -DNDEBUG")
endif()
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Eigen3: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "ZeroMQ: ${ZMQ_LIBRARIES}")
message(STATUS "LibTorch: ${USE_LIBTORCH}")
if(USE_LIBTORCH AND Torch_FOUND)
    message(STATUS "  LibTorch path: ${TORCH_INSTALL_PREFIX}")
endif()
message(STATUS "=====================================================")
message(STATUS "")
message(STATUS "To build Debug version: cmake -DCMAKE_BUILD_TYPE=Debug ..")
message(STATUS "To build Release version: cmake -DCMAKE_BUILD_TYPE=Release ..")
message(STATUS "")

