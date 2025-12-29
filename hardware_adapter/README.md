# Hardware Adapter C Implementation

This directory contains the C implementation of the hardware adapter, converted from the Python version.

## Overview

The hardware adapter acts as a bridge between the system manager and the MAVLink flight controller. It:
- Receives commands from the system manager via ZMQ
- Sends commands to the flight controller via MAVLink
- Receives flight data from the flight controller via MAVLink
- Publishes flight data to the system manager via ZMQ

## Architecture

The implementation uses two threads:
1. **Command Thread**: Handles incoming commands from system_manager and forwards them to MAVLink
2. **Data Thread**: Maintains the current flight data structure and publishes it to system_manager

## Dependencies

- **libzmq**: ZeroMQ library for inter-process communication
- **pthread**: POSIX threads for multi-threading
- **mavlink**: MAVLink C library (v2.0)

### Installing MAVLink C Library

The MAVLink C library is required for compilation. You have two options:

**Option 1: System-wide installation**
```bash
git clone https://github.com/mavlink/mavlink.git
cd mavlink
# Install headers to system directory (requires sudo)
sudo cp -r include/mavlink /usr/include/
```

**Option 2: Local installation (recommended)**
```bash
git clone https://github.com/mavlink/mavlink.git
# Then build with:
make MAVLINK_INCLUDE=-I../mavlink/include
```

Or set it permanently in the Makefile by editing the `MAVLINK_INCLUDE` variable.

## Building

### Install Dependencies

On Ubuntu/Debian:
```bash
make install-deps
```

Or manually:
```bash
sudo apt-get install libzmq3-dev build-essential
```

### Build

**Standard build (if MAVLink is system-wide):**
```bash
make
```

**Build with local MAVLink:**
```bash
make MAVLINK_INCLUDE=-I/path/to/mavlink/include
```

**Debug build:**
```bash
make debug
```

This will create the executable at `bin/hardware_adapter` (or `bin/hardware_adapter_debug` for debug build).

### Clean

```bash
make clean
```

## Usage

```bash
./bin/hardware_adapter [log_directory]
```

If no log directory is specified, it defaults to `../logs/`.

## Implementation Notes

### MAVLink Integration

The implementation includes full MAVLink support:

1. **Connection**: Parses address string (format: `udp:IP:PORT` or `udp:PORT`) and creates UDP socket
2. **Message Reception**: Receives and parses MAVLink messages from the flight controller
3. **Message Sending**: Sends MAVLink commands (heartbeat, setpoints, attitude, commands)
4. **Message Parsing**: Parses common MAVLink messages:
   - HEARTBEAT
   - ATTITUDE_QUATERNION
   - ATTITUDE
   - LOCAL_POSITION_NED
   - GLOBAL_POSITION_INT
   - HIGHRES_IMU
   - ALTITUDE
   - VFR_HUD

The connection uses UDP sockets and supports non-blocking I/O for efficient message handling.

### Serialization

The current implementation uses a simplified serialization (direct memory copy) which is not portable across different architectures. For production use, consider:

- **MessagePack**: Binary serialization format
- **Protocol Buffers**: Google's serialization format
- **JSON**: Text-based format (slower but more portable)

Update the serialization functions:
- `flight_data_serialize()`
- `flight_data_deserialize()`
- `deserialize_attitude_cmd()`
- `deserialize_vel_cmd()`

### ZMQ Topics

The ZMQ topics are defined in `zmq_topics.h` and match the Python implementation:
- Publisher port: 7790
- Subscriber port: 7793

## File Structure

```
c/
├── common.h              - Common data structures and utilities
├── common.c              - Implementation of common functions
├── low_pass_filter.h     - Low pass filter header
├── low_pass_filter.c     - Low pass filter implementation
├── zmq_topics.h          - ZMQ topic definitions
├── zmq_wrapper.h         - ZMQ wrapper header
├── zmq_wrapper.c         - ZMQ wrapper implementation
├── hardware_adapter.h    - Hardware adapter header
├── hardware_adapter.c    - Hardware adapter implementation
├── main.c                - Main entry point
├── Makefile              - Build configuration
└── README.md             - This file
```

## Differences from Python Version

1. **Static Typing**: All types are explicitly defined
2. **Manual Memory Management**: Memory must be explicitly allocated and freed
3. **Threading**: Uses pthreads instead of Python's threading module
4. **Serialization**: Uses simplified binary format (needs proper implementation)
5. **MAVLink**: Uses placeholder functions (needs MAVLink C library integration)

## TODO

- [ ] Integrate MAVLink C library
- [ ] Implement proper message parsing
- [ ] Implement proper serialization (MessagePack/Protobuf)
- [ ] Add error handling and logging
- [ ] Add unit tests
- [ ] Add configuration file support

