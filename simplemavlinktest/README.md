# simpleMavlinkTest (C version)

C implementation of simpleMavlinkTest for replaying MAVLink commands from hardware_adapter CSV logs or generating sine-wave velocity commands.

## Features

- **Log Replay**: Replay SET_POSITION_TARGET_LOCAL_NED commands from hardware_adapter CSV log files
- **Sine Test**: Generate continuous sine-wave velocity (X,Y,Z) and yaw-rate commands
- **Dual Backend**: Support for both ZMQ (default) and direct MAVLink UDP communication
- **Mode Control**: Automatically sets OFFBOARD mode before test and HOLD mode after (MAVLink mode only)

## Building

```bash
cd simplemavlinktest
make
```

## Dependencies

- MAVLink C library headers (should be in `../hardware_adapter/mavlink/` or system-wide)
- ZMQ library (`libzmq3-dev` on Ubuntu/Debian)
- Standard C libraries (pthread, math, etc.)

## Usage

### Sine Test (Default)

```bash
# Sine test via ZMQ (default)
./simpleMavlinkTest --sine

# Sine test via direct MAVLink UDP
./simpleMavlinkTest --sine --direct-mavlink

# Custom duration and frequency
./simpleMavlinkTest --sine --sine-duration 60 --sine-cmd-freq 50
```

### Log Replay

```bash
# Replay log file via ZMQ
./simpleMavlinkTest --log /path/to/log.csv

# Replay at 2x speed, looping continuously
./simpleMavlinkTest --log /path/to/log.csv --loop --speed 2.0

# Replay via direct MAVLink UDP
./simpleMavlinkTest --log /path/to/log.csv --direct-mavlink
```

## Command Line Options

- `--log FILE` - Path to hardware_adapter CSV log for replay
- `--sine` - Send sine-wave velocity commands (default if no --log)
- `--direct-mavlink` - Use direct MAVLink UDP instead of ZMQ (default: ZMQ)
- `--loop` - Loop the log file continuously
- `--speed FACTOR` - Replay speed factor (default: 1.0, 0 = as fast as possible)
- `--sine-duration SEC` - Sine test duration in seconds (default: 20.0)
- `--sine-cmd-freq HZ` - Command frequency for sine test (default: 20.0)
- `--udp-address IP` - UDP address for MAVLink (default: 127.0.0.1)
- `--udp-port PORT` - UDP port for MAVLink (default: 14540)
- `--target-system ID` - Target system ID (default: 1)
- `--target-component ID` - Target component ID (default: 1)
- `--frame FRAME` - MAV_FRAME value (default: 1 = LOCAL_NED)
- `-h, --help` - Show help message

## Examples

```bash
# Default: sine test via ZMQ
./simpleMavlinkTest

# Replay log file
./simpleMavlinkTest --log ../logs/2026-01-06_22-25-40_hardware_adapter.csv

# Fast replay (no timing delays)
./simpleMavlinkTest --log log.csv --speed 0

# Sine test with custom parameters
./simpleMavlinkTest --sine --sine-duration 60 --sine-cmd-freq 50

# Direct MAVLink connection
./simpleMavlinkTest --sine --direct-mavlink --udp-address 192.168.1.100 --udp-port 14550
```

## Notes

- **ZMQ Mode (Default)**: Commands are sent to hardware_adapter via ZMQ on port 7793, topic `quadVelNedCmd`
- **MAVLink Mode**: Direct UDP connection to autopilot, automatically sets OFFBOARD mode before test
- **CSV Format**: Log files must have columns: timestamp, local_ts, command_type, vel_x, vel_y, vel_z, yaw, yaw_rate
- **Sine Test**: Generates sine waves with configurable amplitudes and frequencies for testing

## Differences from Python Version

- C implementation for better performance
- Uses hardware_adapter's ZMQ wrapper and MAVLink connection code
- Same command serialization format for compatibility
- Simplified CSV parsing (no full CSV library dependency)
