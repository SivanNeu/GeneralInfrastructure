#!/usr/bin/env python3
"""
Simple script to connect to a drone via UDP and send arm and takeoff commands.
Uses pymavlink library to communicate with the drone on UDP port (default: 14541).
"""

import sys
import time
from pymavlink import mavutil

def connect_to_drone(udp_port):
    """Connect to the drone via UDP."""
    udp_address = f"127.0.0.1:{udp_port}"
    print(f"Connecting to drone on UDP {udp_address}...")
    try:
        master = mavutil.mavlink_connection(f"udp:{udp_address}")
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"Heartbeat received! System ID: {master.target_system}, Component ID: {master.target_component}")
        return master
    except Exception as e:
        print(f"Error connecting to drone: {e}")
        raise

def takeoff_px4(master, takeoff_altitude=10.0):
    """
    Takeoff function following the exact pattern provided.
    Order: Mode change -> Takeoff command -> Arm
    """
    print("Connected to PX4 autopilot")
    print(master.mode_mapping())
    mode_id = master.mode_mapping()["TAKEOFF"][1]
    print(mode_id)
    
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    starting_alt = msg.alt / 1000
    takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]
    time.sleep(1)
    
    # Change mode to takeoff (PX4)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")
    time.sleep(1)
    
    # Command Takeoff
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        takeoff_params[0],
        takeoff_params[1],
        takeoff_params[2],
        takeoff_params[3],
        takeoff_params[4],
        takeoff_params[5],
        takeoff_params[6]
    )
    
    time.sleep(1)
    
    # Arm the UAS
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    arm_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")
    
    time.sleep(15)
    print("Takeoff done")
    return True

def main():
    """Main function to connect, arm, and takeoff."""
    # Parse command line arguments
    # Default UDP port
    udp_port = 14541
    
    # Check for --udp:portname format
    for arg in sys.argv[1:]:
        if arg.startswith("--udp:"):
            try:
                udp_port = int(arg.split(":", 1)[1])
            except (ValueError, IndexError):
                print(f"Error: Invalid UDP port format: {arg}")
                print("Usage: --udp:PORT (e.g., --udp:14541)")
                sys.exit(1)
        elif arg in ["-h", "--help"]:
            print("Usage: arm_and_takeoff.py [--udp:PORT]")
            print("  --udp:PORT    UDP port to connect to (default: 14541)")
            print("  -h, --help    Show this help message")
            sys.exit(0)
    
    try:
        # Connect to drone
        master = connect_to_drone(udp_port)
        
        # Small delay to ensure connection is stable
        time.sleep(1)
        
        # Use PX4 takeoff pattern from takeoff.py
        # Order: Mode change -> Arm -> Takeoff
        if not takeoff_px4(master, takeoff_altitude=10.0):
            print("Takeoff sequence failed. Exiting.")
            return
        
        print("\nTakeoff sequence completed successfully!")
        print("Monitor the drone's status to confirm takeoff.")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
    except Exception as e:
        print(f"Error: {e}")
        raise

if __name__ == "__main__":
    main()
