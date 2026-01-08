#!/usr/bin/env python3
"""
simpleMavlinkTest.py

Replay MAVLink SET_POSITION_TARGET_LOCAL_NED commands from a hardware_adapter
CSV log file to UDP port 14540, or generate sine-wave velocity/yaw-rate
commands for testing.

Requirements:
    pip install pymavlink

Usage (log replay):
    python simpleMavlinkTest.py --log /path/to/log.csv [--speed 1.0] [--loop]

Usage (sine test, no input CSV needed):
    python simpleMavlinkTest.py --sine [--sim-time 60] [--sine-cmd-freq 50]
"""

import argparse
import csv
import math
import socket
import struct
import time
from typing import Optional

import numpy as np
from pymavlink import mavutil

import zmqTopics
import zmqWrapper


# =========================
# Sine input configuration
# =========================

# Default simulation time for synthetic (sine) data generation (seconds)
SIM_TIME = 20.0

# Amplitudes (in m/s for velocities, rad/s for yaw rate)
SINE_VX_AMPL = 5.0
SINE_VY_AMPL = 5
SINE_VZ_AMPL = 1
SINE_YAWRATE_AMPL = 3.0

# Frequencies (in Hz) for each channel
SINE_VX_FREQ = 1
SINE_VY_FREQ = 1
SINE_VZ_FREQ = 0.2
SINE_YAWRATE_FREQ = 1

# Command frequency (Hz) - how often to send MAVLink commands
SINE_CMD_FREQ = 50.0  # 50 Hz = 20ms between commands


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay MAVLink SET_POSITION_TARGET_LOCAL_NED from hardware_adapter CSV log"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Replay speed factor (1.0 = real time, 2.0 = 2x faster, 0 = no timing)",
    )
    parser.add_argument(
        "--target-system",
        type=int,
        default=1,
        help="Target system id (default: 1)",
    )
    parser.add_argument(
        "--target-component",
        type=int,
        default=1,
        help="Target component id (default: 1, typically autopilot)",
    )
    parser.add_argument(
        "--frame",
        type=int,
        default=1,
        help="MAV_FRAME value (default: 1 = MAV_FRAME_LOCAL_NED)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop the log file continuously",
    )
    parser.add_argument(
        "--udp-address",
        default="127.0.0.1",
        help="UDP address for MAVLink (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=14540,
        help="UDP port for MAVLink (default: 14540)",
    )
    # Log file:
    # - In replay mode: input CSV with logged setpoints (required)
    # - In sine mode: optional, currently unused (no CSV needed)
    parser.add_argument(
        "--log",
        dest="logfile",
        type=str,
        default=None,
        help="Path to hardware_adapter CSV log for replay (required when not using --sine)",
    )
    parser.add_argument(
        "--sine",
        action="store_true",
        help="Send sine-wave velocity (X,Y,Z) and yaw-rate commands instead of log replay",
    )
    # Simulation time length for synthetic (sine) data generation
    parser.add_argument(
        "--sine-duration",
        "--sim-time",
        type=float,
        default=SIM_TIME,
        help=f"Simulation time length in seconds for sine test (default: {SIM_TIME} s)",
    )
    parser.add_argument(
        "--sine-cmd-freq",
        type=float,
        default=None,
        help=f"Command frequency in Hz for sine test (default: {SINE_CMD_FREQ} Hz)",
    )
    parser.add_argument(
        "--direct-mavlink",
        action="store_true",
        help="Use direct MAVLink UDP instead of default ZMQ connection",
    )
    return parser.parse_args()


def open_mavlink_connection(address: str, port: int) -> mavutil.mavfile:
    # Use bidirectional UDP connection to receive heartbeats and verify mode changes
    # Format: "udp:ip:port" for bidirectional, "udpout:ip:port" is send-only
    conn_string = f"udp:{address}:{port}"
    print(f"Connecting to MAVLink UDP endpoint {conn_string}")
    master = mavutil.mavlink_connection(conn_string, source_system=255, source_component=191)
    
    # Send our own heartbeat first to identify ourselves
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,  # base_mode
        0,  # custom_mode
        mavutil.mavlink.MAV_STATE_ACTIVE
    )
    
    # Wait for a heartbeat to verify connection
    print("Waiting for heartbeat from autopilot...")
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg is None:
        print("Warning: No heartbeat received. Connection may not be established.")
        print("Make sure the autopilot is running and sending heartbeats on this port.")
    else:
        print(f"Heartbeat received from system {msg.get_srcSystem()}, component {msg.get_srcComponent()}")
        # Update target system/component from heartbeat
        master.target_system = msg.get_srcSystem()
        master.target_component = msg.get_srcComponent()
    
    return master


def serialize_vel_cmd(vel_cmd, yaw_cmd, yaw_rate_cmd) -> bytes:
    """
    Serialize velocity command to binary format for C code (same format as system_manager).
    Format: magic (4 bytes), version (4 bytes), vel[3] (24 bytes), yaw (8 bytes),
            yaw_rate (8 bytes), yaw_valid (1 byte), yaw_rate_valid (1 byte), padding (2 bytes)
    """
    # Magic number: "VELC" = 0x56454C43
    magic = 0x56454C43
    version = 1

    # Check if yaw/yaw_rate are valid (not NaN)
    yaw_valid = 1 if not math.isnan(yaw_cmd) else 0
    yaw_rate_valid = 1 if not math.isnan(yaw_rate_cmd) else 0

    # Use NaN-safe values
    yaw_val = yaw_cmd if not math.isnan(yaw_cmd) else 0.0
    yaw_rate_val = yaw_rate_cmd if not math.isnan(yaw_rate_cmd) else 0.0

    # Ensure vel_cmd is indexable (x, y, z)
    vx, vy, vz = float(vel_cmd[0]), float(vel_cmd[1]), float(vel_cmd[2])

    # Pack as little-endian binary: '<II' (magic, version), '<3d' (vel), '<2d' (yaw, yaw_rate), '<2B' (flags)
    return (
        struct.pack("<II", magic, version)
        + struct.pack("<3d", vx, vy, vz)
        + struct.pack("<2d", yaw_val, yaw_rate_val)
        + struct.pack("<2B", yaw_valid, yaw_rate_valid)
        + b"\x00\x00"  # padding
    )


def set_mode_offboard(master: mavutil.mavfile, target_system: int, target_component: int) -> None:
    """
    Set flight mode to OFFBOARD using MAV_CMD_DO_SET_MODE.
    For PX4: param1=1.0 (base_mode), param2=6.0 (OFFBOARD custom_mode)
    """
    # Use target system/component from master if available, otherwise use provided values
    ts = master.target_system if master.target_system > 0 else target_system
    tc = master.target_component if master.target_component > 0 else target_component
    
    print(f"Setting mode to OFFBOARD (target: system={ts}, component={tc})...")
    # MAV_CMD_DO_SET_MODE = 176
    # param1: base_mode (MAV_MODE_FLAG), param2: custom_mode (PX4 mode number)
    # Send command multiple times to ensure it's received
    for i in range(5):
        master.mav.command_long_send(
            ts,
            tc,
            176,  # MAV_CMD_DO_SET_MODE
            0,  # confirmation
            1.0,  # param1: base_mode
            6.0,  # param2: custom_mode (OFFBOARD = 6 for PX4)
            0.0,  # param3
            0.0,  # param4
            0.0,  # param5
            0.0,  # param6
            0.0,  # param7
        )
        time.sleep(0.1)
    
    # Wait and check for mode change confirmation via heartbeat
    print("Waiting for OFFBOARD mode confirmation...")
    start_time = time.time()
    timeout = 5.0
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.5)
        if msg is not None:
            # PX4 OFFBOARD mode custom_mode = 393216 (0x60000)
            if msg.custom_mode == 393216:
                print("✓ OFFBOARD mode confirmed!")
                return
            else:
                print(f"  Current mode: {msg.custom_mode} (waiting for 393216)...")
        time.sleep(0.1)
    
    print("Warning: OFFBOARD mode not confirmed after 5 seconds, but continuing anyway...")
    print("  The autopilot may not have received the command or may require additional setup.")


def set_mode_hold(master: mavutil.mavfile, target_system: int, target_component: int) -> None:
    """
    Set flight mode to HOLD using MAV_CMD_DO_SET_MODE.
    For PX4: param1=1.0 (base_mode), param2=4.0 (HOLD custom_mode)
    """
    # Use target system/component from master if available, otherwise use provided values
    ts = master.target_system if master.target_system > 0 else target_system
    tc = master.target_component if master.target_component > 0 else target_component
    
    print(f"Setting mode to HOLD (target: system={ts}, component={tc})...")
    # MAV_CMD_DO_SET_MODE = 176
    # param1: base_mode (MAV_MODE_FLAG), param2: custom_mode (PX4 mode number)
    # Send command multiple times to ensure it's received
    for i in range(5):
        master.mav.command_long_send(
            ts,
            tc,
            176,  # MAV_CMD_DO_SET_MODE
            0,  # confirmation
            1.0,  # param1: base_mode
            4.0,  # param2: custom_mode (HOLD = 4 for PX4)
            0.0,  # param3
            0.0,  # param4
            0.0,  # param5
            0.0,  # param6
            0.0,  # param7
        )
        time.sleep(0.1)
    
    # Wait and check for mode change confirmation via heartbeat
    print("Waiting for HOLD mode confirmation...")
    start_time = time.time()
    timeout = 5.0
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.5)
        if msg is not None:
            # PX4 HOLD mode custom_mode = 50593792 (0x3040000)
            if msg.custom_mode == 50593792:
                print("✓ HOLD mode confirmed!")
                return
            else:
                print(f"  Current mode: {msg.custom_mode} (waiting for 50593792)...")
        time.sleep(0.1)
    
    print("Warning: HOLD mode not confirmed after 5 seconds, but continuing anyway...")


def replay_log_once(
    master: Optional[mavutil.mavfile],
    logfile: str,
    speed: float,
    target_system: int,
    target_component: int,
    frame: int,
    use_zmq: bool = False,
    zmq_pub=None,
) -> None:
    """
    Replay all SETPOINT entries from the CSV log once.
    """
    with open(logfile, "r", newline="") as f:
        reader = csv.DictReader(f)

        # Ensure expected columns exist
        required_cols = {
            "timestamp",
            "local_ts",
            "command_type",
            "pos_x",
            "pos_y",
            "pos_z",
            "vel_x",
            "vel_y",
            "vel_z",
            "acc_x",
            "acc_y",
            "acc_z",
            "yaw",
            "yaw_rate",
        }
        missing = required_cols - set(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"Log file is missing required columns: {', '.join(sorted(missing))}")

        prev_local_ts: Optional[float] = None
        start_wall_time: Optional[float] = None
        sent_count = 0
        last_print_time = time.time()

        for row in reader:
            if row.get("command_type") != "SETPOINT":
                continue

            try:
                timestamp_s = float(row["timestamp"])
                local_ts = float(row["local_ts"])
                pos_x = float(row["pos_x"])
                pos_y = float(row["pos_y"])
                pos_z = float(row["pos_z"])
                vel_x = float(row["vel_x"])
                vel_y = float(row["vel_y"])
                vel_z = float(row["vel_z"])
                acc_x = float(row["acc_x"])
                acc_y = float(row["acc_y"])
                acc_z = float(row["acc_z"])
                yaw = float(row["yaw"])
                yaw_rate = float(row["yaw_rate"])
            except (ValueError, KeyError) as e:
                print(f"Skipping malformed row: {e}")
                continue

            # hardware_adapter logs timestamp in seconds (time_boot_ms / 1000.0)
            time_boot_ms = int(timestamp_s * 1000.0)

            # Timing control: maintain exact frequency from log file
            if speed > 0.0:
                if prev_local_ts is None:
                    # First message: start timing from now
                    prev_local_ts = local_ts
                    start_wall_time = time.time()
                else:
                    # Calculate interval between consecutive messages
                    dt_log = local_ts - prev_local_ts
                    if dt_log > 0:
                        # Sleep for the interval (adjusted by speed factor)
                        sleep_time = dt_log / max(speed, 1e-6)
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                    prev_local_ts = local_ts
            else:
                # speed == 0: send as fast as possible (no timing)
                prev_local_ts = local_ts

            # Build type_mask: we use velocity + yaw/yaw_rate, ignore position and acceleration
            # Bits: 0-2 position, 3-5 velocity, 6-8 acceleration, 10 yaw, 11 yaw_rate
            # 1 = ignore, 0 = use
            type_mask = (
                0x07  # ignore position x,y,z
                | 0x1C0  # ignore acceleration x,y,z
            )

            if use_zmq:
                # Send via ZMQ to hardware_adapter (velocity NED command)
                if zmq_pub is None:
                    raise RuntimeError("ZMQ publisher socket is None while use_zmq=True")
                vel_cmd = (vel_x, vel_y, vel_z)
                cmd_data = serialize_vel_cmd(vel_cmd, yaw, yaw_rate)
                zmq_pub.send_multipart([zmqTopics.topicGuidenceCmdVelNed, cmd_data])
            else:
                if master is None:
                    raise RuntimeError("MAVLink master is None while use_zmq=False")
                # Use target system/component from master if available
                ts = master.target_system if master.target_system > 0 else target_system
                tc = master.target_component if master.target_component > 0 else target_component

                master.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    ts,
                    tc,
                    frame,
                    type_mask,
                    pos_x,
                    pos_y,
                    pos_z,
                    vel_x,
                    vel_y,
                    vel_z,
                    acc_x,
                    acc_y,
                    acc_z,
                    yaw,
                    yaw_rate,
                )

            sent_count += 1

            # Periodically print current command (about once per second)
            now_print = time.time()
            if now_print - last_print_time >= 1.0:
                print(
                    f"Replay cmd #{sent_count}: "
                    f"vel=({vel_x:.3f}, {vel_y:.3f}, {vel_z:.3f}) m/s, "
                    f"yaw={yaw:.3f} rad, yaw_rate={yaw_rate:.3f} rad/s"
                )
                last_print_time = now_print

            # Periodically check for heartbeats and send our own heartbeat to maintain connection
            # This also helps maintain the connection
            if (not use_zmq) and master is not None and sent_count % 10 == 0:
                # Send our heartbeat
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,  # base_mode
                    0,  # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )

                # Check for incoming heartbeat
                msg = master.recv_match(type="HEARTBEAT", blocking=False, timeout=0)
                if msg is not None and sent_count % 50 == 0:
                    print(f"Sent {sent_count} messages, current mode: {msg.custom_mode}")

        print(f"Finished replaying log {logfile}, sent {sent_count} SETPOINT messages")


def run_sine_test(
    master: Optional[mavutil.mavfile],
    speed: float,
    target_system: int,
    target_component: int,
    frame: int,
    duration: Optional[float] = None,
    cmd_freq: Optional[float] = None,
    use_zmq: bool = False,
    zmq_pub=None,
) -> None:
    """
    Send continuous sine-wave velocity (X,Y,Z) and yaw-rate commands.
    
    Args:
        cmd_freq: Command frequency in Hz (how often to send MAVLink commands).
                  If None, uses SINE_CMD_FREQ from configuration.
    """
    ts = None
    tc = None
    if not use_zmq:
        if master is None:
            raise RuntimeError("MAVLink master is None while use_zmq=False in run_sine_test")
        ts = master.target_system if master.target_system > 0 else target_system
        tc = master.target_component if master.target_component > 0 else target_component
    else:
        if zmq_pub is None:
            raise RuntimeError("ZMQ publisher socket is None while use_zmq=True in run_sine_test")

    # Use provided cmd_freq or default from configuration
    freq = cmd_freq if cmd_freq is not None else SINE_CMD_FREQ
    dt = 1.0 / freq  # Time step in seconds

    print(
        f"Starting sine test with amplitudes "
        f"vx={SINE_VX_AMPL} m/s, vy={SINE_VY_AMPL} m/s, vz={SINE_VZ_AMPL} m/s, "
        f"yaw_rate={SINE_YAWRATE_AMPL} rad/s"
    )
    print(f"Command frequency: {freq} Hz (dt = {dt*1000:.2f} ms)")

    # Type mask: use velocity and yaw_rate, ignore position, acceleration, and yaw angle
    # Bits: 0-2 position, 3-5 velocity, 6-8 acceleration, 10 yaw, 11 yaw_rate
    # 1 = ignore, 0 = use
    type_mask = 0x07 | 0x1C0 | 0x400  # ignore pos, acc, yaw; use vel and yaw_rate

    start_time = time.time()
    last_heartbeat_time = start_time
    sent_count = 0
    last_print_time = start_time

    try:
        while True:
            now = time.time()
            t = now - start_time

            if duration is not None and t >= duration:
                break

            # Compute sine values
            vx = SINE_VX_AMPL * math.sin(2.0 * math.pi * SINE_VX_FREQ * t)
            vy = SINE_VY_AMPL * math.sin(2.0 * math.pi * SINE_VY_FREQ * t)
            vz = SINE_VZ_AMPL * math.sin(2.0 * math.pi * SINE_VZ_FREQ * t)
            yaw_rate = SINE_YAWRATE_AMPL * math.sin(2.0 * math.pi * SINE_YAWRATE_FREQ * t)

            # We don't use position or acceleration or yaw angle in this test
            pos_x = pos_y = pos_z = 0.0
            acc_x = acc_y = acc_z = 0.0
            yaw = 0.0

            # time_boot_ms: relative time since start of test
            time_boot_ms = int(t * 1000.0)

            if use_zmq:
                # Send via ZMQ as velocity NED command
                vel_cmd = (vx, vy, vz)
                cmd_data = serialize_vel_cmd(vel_cmd, yaw, yaw_rate)
                zmq_pub.send_multipart([zmqTopics.topicGuidenceCmdVelNed, cmd_data])
            else:
                master.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    ts,
                    tc,
                    frame,
                    type_mask,
                    pos_x,
                    pos_y,
                    pos_z,
                    vx,
                    vy,
                    vz,
                    acc_x,
                    acc_y,
                    acc_z,
                    yaw,
                    yaw_rate,
                )

            sent_count += 1

            # Periodically print current command (about once per second)
            if now - last_print_time >= 1.0:
                print(
                    f"Sine cmd #{sent_count}: "
                    f"vel=({vx:.3f}, {vy:.3f}, {vz:.3f}) m/s, "
                    f"yaw_rate={yaw_rate:.3f} rad/s"
                )
                last_print_time = now

            # Periodically send and check heartbeats to keep connection alive (MAVLink mode only)
            if (not use_zmq) and master is not None and now - last_heartbeat_time >= 1.0:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                msg = master.recv_match(type="HEARTBEAT", blocking=False, timeout=0)
                if msg is not None and sent_count % 50 == 0:
                    print(f"Sine: sent {sent_count} messages, current mode: {msg.custom_mode}")
                last_heartbeat_time = now

            # Sleep according to desired update rate and speed factor
            sleep_dt = dt / max(speed, 1e-6) if speed > 0.0 else dt
            time.sleep(sleep_dt)

    except KeyboardInterrupt:
        print("Sine test interrupted by user.")

    print(f"Sine test finished, sent {sent_count} SETPOINT messages")


def main() -> None:
    args = parse_args()

    # If no mode explicitly selected, default to synthetic (sine) mode
    # i.e., running without --sine and without --log will use sine generation.
    if not args.sine and not args.logfile:
        print("No mode specified, defaulting to synthetic sine test (--sine).")
        args.sine = True

    # Decide backend: ZMQ is default, direct MAVLink only if explicitly requested
    use_zmq = not args.direct_mavlink

    # If using ZMQ, we don't need a MAVLink connection; otherwise, initialize MAVLink
    master = None
    zmq_pub = None

    if use_zmq:
        # Create ZMQ publisher to send commands to hardware_adapter
        zmq_pub = zmqWrapper.publisher(zmqTopics.topicGuidenceCmdPort, ip="*")
        print(
            f"Sending commands via ZMQ on port {zmqTopics.topicGuidenceCmdPort}, "
            f"topic {zmqTopics.topicGuidenceCmdVelNed!r}"
        )
    else:
        # Simple check that UDP port is usable (not strictly required)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("", 0))
            sock.close()
        except OSError as e:
            print(f"Warning: UDP socket check failed: {e}")

        master = open_mavlink_connection(args.udp_address, args.udp_port)

    try:
        if (not use_zmq) and master is not None:
            # Set mode to OFFBOARD before starting replay (MAVLink mode only)
            set_mode_offboard(master, args.target_system, args.target_component)

        if args.sine:
            # Run sine-wave test instead of log replay
            run_sine_test(
                master=master,
                speed=args.speed,
                target_system=args.target_system,
                target_component=args.target_component,
                frame=args.frame,
                duration=args.sine_duration,
                cmd_freq=args.sine_cmd_freq,
                use_zmq=use_zmq,
                zmq_pub=zmq_pub,
            )
        else:
            # Default: replay log file
            if not args.logfile:
                raise RuntimeError("Replay mode requires --log to be specified")
            while True:
                replay_log_once(
                    master=master,
                    logfile=args.logfile,
                    speed=args.speed,
                    target_system=args.target_system,
                    target_component=args.target_component,
                    frame=args.frame,
                    use_zmq=use_zmq,
                    zmq_pub=zmq_pub,
                )
                if not args.loop:
                    break
    finally:
        # Set mode back to HOLD after replay completes (MAVLink mode only)
        if (not use_zmq) and master is not None:
            try:
                set_mode_hold(master, args.target_system, args.target_component)
            except Exception as e:
                print(f"Warning: Failed to set mode to HOLD: {e}")

            if hasattr(master, "close"):
                try:
                    master.close()
                except Exception:
                    pass

        # Close ZMQ publisher if used
        if zmq_pub is not None:
            try:
                zmq_pub.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()

