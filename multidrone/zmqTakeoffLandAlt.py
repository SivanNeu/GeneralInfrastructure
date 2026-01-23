#!/usr/bin/env python3
"""
Simple script to send a takeoff or land command to the drone via ZMQ.
Connects to the hardware_adapter and sends:
 - Takeoff command with specified altitude.
 - Land command.
 - Altitude change command using OFFBOARD mode with 10Hz position setpoints.

Can subscribe to drone state data from mavlink_to_ZMQ via --zmqin.
"""

import zmq
import struct
import time
import sys
import argparse
import threading

# ZMQ configuration
ZMQ_CMD_PORT_DEFAULT = 7700   # Port to SEND commands to zmq_commands_mavlink
ZMQ_STATE_PORT_DEFAULT = 9900 # Port to RECEIVE state from mavlink_to_ZMQ

# Command topics
TAKEOFF_TOPIC = b'quadTakeoffCmd'
LAND_TOPIC = b'quadLandCmd'
ALTITUDE_TOPIC = b'quadAltCmd'
POS_NED_TOPIC = b'quadPosNedCmd'
MODE_TOPIC = b'quadModeCmd'
FLIGHT_DATA_TOPIC = b'FLIGHT_DATA'

# PX4 Mode IDs (custom_mode values)
MODE_ID_OFFBOARD = 393216
MODE_ID_HOLD = 50593792
MODE_ID_RETURN = 84148224
MODE_ID_TAKEOFF = 33816576
MODE_ID_LAND = 100925440
MODE_ID_GENERAL = 65535
MODE_ID_AUTO = 262144
MODE_ID_ACRO = 327680
MODE_ID_RATTITUDE = 524288
MODE_ID_ALTITUDE = 131072
MODE_ID_POSITION = 196608
MODE_ID_LOITER = 262147
MODE_ID_MISSION = 262148
MODE_ID_MANUAL = 65536
MODE_ID_STABILIZED = 458752
MODE_ID_POSITION_SLOW = 33751040
MODE_ID_SAFE_RECOVERY = 84148224
MODE_ID_FOLLOW_TARGET = 134479872
MODE_ID_PRECISION_LAND = 151257088

# Mode ID to name mapping
MODE_NAMES = {
    MODE_ID_OFFBOARD: "OFFBOARD",
    MODE_ID_HOLD: "HOLD",
    MODE_ID_RETURN: "RETURN",
    MODE_ID_TAKEOFF: "TAKEOFF",
    MODE_ID_LAND: "LAND",
    MODE_ID_GENERAL: "GENERAL",
    MODE_ID_AUTO: "AUTO",
    MODE_ID_ACRO: "ACRO",
    MODE_ID_RATTITUDE: "RATTITUDE",
    MODE_ID_ALTITUDE: "ALTITUDE",
    MODE_ID_POSITION: "POSITION",
    MODE_ID_LOITER: "LOITER",
    MODE_ID_MISSION: "MISSION",
    MODE_ID_MANUAL: "MANUAL",
    MODE_ID_STABILIZED: "STABILIZED",
    MODE_ID_POSITION_SLOW: "POSITION_SLOW",
    MODE_ID_FOLLOW_TARGET: "FOLLOW_TARGET",
    MODE_ID_PRECISION_LAND: "PRECISION_LAND",
}

# Flight data format constants
FLIGHT_DATA_MAGIC = 0x464C4947  # "FLIG"
FLIGHT_DATA_VERSION = 1
FLIGHT_DATA_NUM_DOUBLES = 63  # Actual count from C code dbl_fields array (lines 1017-1079)

# Field indices in the double array (0-indexed, matching C serialization order)
# 0-18: quat_ts, imu_ts, timestamp, local_ts, temperature, amsl_m, local_m, monotonic_m,
#       relative_m, terrain_m, bottom_clearance_m, pressure, absolute_press_hpa,
#       differential_press_hpa, signal_strength_percent, throttle, heading, groundspeed, current_thrust
# 19-23: quat x,y,z,w, timestamp
# 24-27: altitude_m (amsl, relative, vertical_speed_estimate, timestamp)
# 28-34: imu_raw_frd (accel[3], gyro[3], timestamp)
# 35-41: imu_ned (accel[3], gyro[3], timestamp)
# 42-48: pos_ned_m (ned[3], vel_ned[3], timestamp)
# 49-52: raw_pos_lla_deg (lla[3], timestamp)
# 53-56: filt_pos_lla_deg (lla[3], timestamp)
# 57-59: rpy_rates[3]
# 60-62: rpy[3]
IDX_TIMESTAMP = 2
IDX_LOCAL_TS = 3
IDX_RELATIVE_M = 8
IDX_ALTITUDE_RELATIVE = 25
IDX_POS_NED_X = 42
IDX_POS_NED_Y = 43
IDX_POS_NED_Z = 44


class DroneState:
    """Parsed drone state from flight data message."""
    def __init__(self):
        self.message_count = 0
        self.timestamp = 0.0
        self.local_ts = 0.0
        self.relative_altitude_m = 0.0
        self.pos_ned_x = 0.0
        self.pos_ned_y = 0.0
        self.pos_ned_z = 0.0
        self.custom_mode_id = 0
        self.valid = False
    
    def mode_name(self):
        """Return human-readable mode name with ID."""
        name = MODE_NAMES.get(self.custom_mode_id, "UNKNOWN")
        return f"{name}({self.custom_mode_id})"
    
    def __repr__(self):
        return f"DroneState(alt={self.relative_altitude_m:.2f}m, pos=({self.pos_ned_x:.2f}, {self.pos_ned_y:.2f}, {self.pos_ned_z:.2f}), mode={self.mode_name()})"


def parse_flight_data(data: bytes) -> DroneState:
    """Parse binary flight data from mavlink_to_ZMQ."""
    state = DroneState()
    # Header(12) + 61 doubles(488) + custom_mode_id(4) + mode(4) + bools(1) + gathered(4) = 513 bytes min
    min_len = 12 + FLIGHT_DATA_NUM_DOUBLES * 8 + 4  # At least need custom_mode_id
    if len(data) < min_len:
        return state
    
    magic, version, msg_count = struct.unpack_from('<III', data, 0)
    if magic != FLIGHT_DATA_MAGIC or version != FLIGHT_DATA_VERSION:
        return state
    
    doubles = struct.unpack_from(f'<{FLIGHT_DATA_NUM_DOUBLES}d', data, 12)
    
    # Parse uint32 fields after doubles (offset = 12 + 61*8 = 500)
    uint_offset = 12 + FLIGHT_DATA_NUM_DOUBLES * 8
    custom_mode_id, = struct.unpack_from('<I', data, uint_offset)
    
    state.message_count = msg_count
    state.timestamp = doubles[IDX_TIMESTAMP]
    state.local_ts = doubles[IDX_LOCAL_TS]
    state.relative_altitude_m = doubles[IDX_ALTITUDE_RELATIVE]
    state.pos_ned_x = doubles[IDX_POS_NED_X]
    state.pos_ned_y = doubles[IDX_POS_NED_Y]
    state.pos_ned_z = doubles[IDX_POS_NED_Z]
    state.custom_mode_id = custom_mode_id
    state.valid = True
    return state


class DroneStateSubscriber:
    """Continuous drone state subscriber running in a background thread."""
    def __init__(self, zmq_port: int):
        self.zmq_port = zmq_port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        self.socket.connect(f"tcp://127.0.0.1:{zmq_port}")
        self.socket.setsockopt(zmq.SUBSCRIBE, FLIGHT_DATA_TOPIC)
        
        self.current_state = DroneState()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
    
    def _run(self):
        while self.running:
            try:
                message = self.socket.recv()
                topic_len = len(FLIGHT_DATA_TOPIC)
                if len(message) > topic_len and message[:topic_len] == FLIGHT_DATA_TOPIC:
                    state = parse_flight_data(message[topic_len:])
                    if state.valid:
                        with self.lock:
                            self.current_state = state
            except zmq.Again:
                pass
            except Exception:
                pass
    
    def get_state(self) -> DroneState:
        with self.lock:
            return self.current_state
    
    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.socket.close()
        self.context.term()


class CommandPublisher:
    """ZMQ publisher for sending commands to hardware_adapter."""
    def __init__(self, zmq_port: int):
        self.zmq_port = zmq_port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{zmq_port}")
        print(f"ZMQ Publisher: Bound to tcp://*:{zmq_port}")
        # Wait for subscriber to connect
        time.sleep(2.0)
    
    def send_pos_ned(self, x: float, y: float, z: float):
        """Send position setpoint in NED frame."""
        data = struct.pack('ddd', x, y, z)
        self.socket.send(POS_NED_TOPIC + data)
    
    def send_mode(self, mode_id: int):
        """Send mode change command."""
        data = struct.pack('I', mode_id)
        self.socket.send(MODE_TOPIC + data)
    
    def send_takeoff(self, altitude: float):
        """Send takeoff command."""
        data = struct.pack('d', altitude)
        self.socket.send(TAKEOFF_TOPIC + data)
    
    def send_land(self):
        """Send land command."""
        data = struct.pack('d', 0.0)
        self.socket.send(LAND_TOPIC + data)
    
    def close(self):
        self.socket.close()
        self.context.term()


def run_altitude_control(target_altitude: float, zmq_in_port: int, zmq_out_port: int, 
                         timeout: float = 30.0, threshold: float = 0.5):
    """
    Run altitude control loop using OFFBOARD mode with position setpoints.
    Preserves X/Y position, only changes Z (altitude).
    """
    print(f"Starting altitude control to {target_altitude}m...")
    print(f"  ZMQ in (state): {zmq_in_port}")
    print(f"  ZMQ out (cmd): {zmq_out_port}")
    
    # Start state subscriber
    subscriber = DroneStateSubscriber(zmq_in_port)
    time.sleep(0.5)  # Let subscriber get initial data
    
    # Get initial state
    state = subscriber.get_state()
    if not state.valid:
        print("Error: Cannot get valid drone state")
        subscriber.stop()
        return False
    
    current_x = state.pos_ned_x
    current_y = state.pos_ned_y
    target_z = -target_altitude  # NED: negative Z is up
    
    print(f"  Current position: ({current_x:.2f}, {current_y:.2f}, {state.pos_ned_z:.2f})")
    print(f"  Target: Z = {target_z:.2f} (altitude {target_altitude:.2f}m)")
    
    # Start command publisher
    publisher = CommandPublisher(zmq_out_port)
    
    # Step 1: Send initial setpoints before switching to OFFBOARD
    print("Sending initial setpoints...")
    for _ in range(20):  # ~2 seconds at 10Hz
        publisher.send_pos_ned(current_x, current_y, target_z)
        time.sleep(0.1)
    
    # Step 2: Switch to OFFBOARD mode
    print("Switching to OFFBOARD mode...")
    publisher.send_mode(MODE_ID_OFFBOARD)
    time.sleep(0.5)
    
    # Verify mode after switch
    state = subscriber.get_state()
    print(f"  Mode after switch: {state.mode_name()} (id={state.custom_mode_id})")
    
    # Step 3: Control loop - send setpoints at 10Hz until altitude reached
    print(f"Waiting for altitude {target_altitude}m (threshold: ±{threshold}m)...")
    start_time = time.time()
    last_status_time = start_time
    reached = False
    loop_count = 0
    
    while time.time() - start_time < timeout:
        # Send position setpoint (preserve X/Y, change Z)
        publisher.send_pos_ned(current_x, current_y, target_z)
        loop_count += 1
        
        # Check current altitude
        state = subscriber.get_state()
        if state.valid:
            current_alt = -state.pos_ned_z  # Convert NED Z to positive altitude
            
            # Print status every 2 seconds
            if time.time() - last_status_time >= 2.0:
                print(f"  [{loop_count}] Alt: {current_alt:.2f}m, Target: {target_altitude:.2f}m, Mode: {state.mode_name()}")
                last_status_time = time.time()
            
            if abs(current_alt - target_altitude) < threshold:
                print(f"✓ Altitude reached: {current_alt:.2f}m (target: {target_altitude:.2f}m)")
                reached = True
                break
        
        time.sleep(0.1)  # 10Hz
    
    if not reached:
        state = subscriber.get_state()
        print(f"Warning: Timeout after {timeout}s, current altitude: {-state.pos_ned_z:.2f}m, mode: {state.mode_name()}")
    
    # Step 4: Switch to HOLD mode to maintain position
    print("Switching to HOLD mode...")
    publisher.send_mode(MODE_ID_HOLD)
    time.sleep(0.5)
    
    # Cleanup
    publisher.close()
    subscriber.stop()
    
    print("Altitude control complete.")
    return reached


def send_simple_command(topic, data, topic_name, zmq_port):
    """Send a one-shot command via ZMQ."""
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.setsockopt(zmq.LINGER, 1000)  # 1s linger to ensure message is sent
    try:
        pub_socket.bind(f"tcp://*:{zmq_port}")
        print(f"ZMQ Publisher: Bound to tcp://*:{zmq_port}")
        time.sleep(2.0)  # Wait for subscriber to connect
        
        print(f"Sending {topic_name}:")
        print(f"  Topic: {topic}")
        print(f"  Data (hex): {data.hex()}")
        pub_socket.send(topic + data)
        print(f"✓ {topic_name} sent successfully!")
        time.sleep(1.0)  # Longer delay to ensure delivery
        return True
    except Exception as e:
        print(f"✗ Error: {e}")
        return False
    finally:
        pub_socket.close()
        context.term()


def main():
    parser = argparse.ArgumentParser(
        description="Send takeoff, land, or altitude commands via ZMQ.",
        epilog="""Examples:
  %(prog)s --takeoff --altitude=15 --zmqcmd=7700
  %(prog)s --land --zmqcmd=7700
  %(prog)s --altitude=20 --zmqstate=9900 --zmqcmd=7700  # Altitude change with feedback
  %(prog)s --state --zmqstate=9900                       # Just read drone state""",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('--takeoff', action='store_true', help='Send takeoff command')
    group.add_argument('--land', action='store_true', help='Send land command')
    group.add_argument('--state', action='store_true', help='Just read and print drone state')

    parser.add_argument('--altitude', type=float, default=10.0, metavar='ALT', 
                        help='Target altitude in meters (default: 10.0)')
    parser.add_argument('--time', dest='timeout', type=float, default=30.0, metavar='SEC',
                        help='Timeout for altitude control (default: 30.0)')
    parser.add_argument('--threshold', type=float, default=0.5, metavar='M',
                        help='Altitude threshold for success (default: 0.5)')
    parser.add_argument('--zmqcmd', dest='zmq_cmd_port', type=int, default=ZMQ_CMD_PORT_DEFAULT,
                        metavar='PORT', help=f'ZMQ port to send commands (default: {ZMQ_CMD_PORT_DEFAULT})')
    parser.add_argument('--zmqstate', dest='zmq_state_port', type=int, default=None, metavar='PORT',
                        help=f'ZMQ port to receive drone state (default: {ZMQ_STATE_PORT_DEFAULT})')
    # Legacy support
    parser.add_argument('--zmq', dest='zmq_legacy', type=int, default=None, 
                        metavar='PORT', help=argparse.SUPPRESS)
    parser.add_argument('--zmqin', dest='zmq_in_legacy', type=int, default=None,
                        metavar='PORT', help=argparse.SUPPRESS)
    parser.add_argument('--zmqout', dest='zmq_out_legacy', type=int, default=None,
                        metavar='PORT', help=argparse.SUPPRESS)

    args = parser.parse_args()
    
    # Handle legacy arguments
    # User's mental model: zmqin = where data goes IN to zmq_commands_mavlink (send commands)
    #                      zmqout = where data comes OUT from mavlink_to_ZMQ (receive state)
    if args.zmq_legacy is not None:
        print("Warning: --zmq is deprecated, use --zmqcmd")
        args.zmq_cmd_port = args.zmq_legacy
    if args.zmq_in_legacy is not None:
        # --zmqin = port where commands go IN to hardware_adapter
        args.zmq_cmd_port = args.zmq_in_legacy
    if args.zmq_out_legacy is not None:
        # --zmqout = port where state comes OUT from mavlink_to_ZMQ  
        args.zmq_state_port = args.zmq_out_legacy

    # State only mode
    if args.state:
        zmq_state = args.zmq_state_port or ZMQ_STATE_PORT_DEFAULT
        print(f"Reading drone state from port {zmq_state}...")
        sub = DroneStateSubscriber(zmq_state)
        time.sleep(1.0)
        state = sub.get_state()
        sub.stop()
        if state.valid:
            print(f"\n✓ Drone State:")
            print(f"  Position NED: ({state.pos_ned_x:.2f}, {state.pos_ned_y:.2f}, {state.pos_ned_z:.2f})")
            print(f"  Altitude: {-state.pos_ned_z:.2f}m")
            sys.exit(0)
        else:
            print("✗ Failed to get drone state")
            sys.exit(1)

    # Determine command
    altitude_provided = any(arg.startswith('--altitude') for arg in sys.argv)
    
    if not (args.takeoff or args.land or altitude_provided):
        parser.print_help()
        sys.exit(0)

    print("=" * 60)
    print("ZMQ Command Sender")
    print("=" * 60)

    if args.takeoff:
        print(f"Command: TAKEOFF to {args.altitude}m")
        success = send_simple_command(TAKEOFF_TOPIC, struct.pack('d', args.altitude),
                                       "Takeoff Command", args.zmq_cmd_port)
    elif args.land:
        print("Command: LAND")
        success = send_simple_command(LAND_TOPIC, struct.pack('d', 0.0),
                                       "Land Command", args.zmq_cmd_port)
    else:
        # Altitude change with feedback control
        if args.zmq_state_port is None:
            print("Error: --zmqstate is required for altitude change")
            print("Use: --altitude=X --zmqstate=PORT --zmqcmd=PORT")
            sys.exit(1)
        
        print(f"Command: SET_ALTITUDE to {args.altitude}m")
        success = run_altitude_control(
            target_altitude=args.altitude,
            zmq_in_port=args.zmq_state_port,
            zmq_out_port=args.zmq_cmd_port,
            timeout=args.timeout,
            threshold=args.threshold
        )

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
