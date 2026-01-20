#!/usr/bin/env python3
"""
Simple script to connect to a drone via UDP and send arm and takeoff commands.
Uses pymavlink library to communicate with the drone on UDP port (default: 14541).
"""

import sys
import time
import argparse
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
    Takeoff function.
    Order: Mode change -> Takeoff command -> Arm
    """
    print("Connected to PX4 autopilot")
    # print(master.mode_mapping())
    try:
        mode_id = master.mode_mapping()["TAKEOFF"][1]
    except KeyError:
        print("Error: TAKEOFF mode not supported by this autopilot.")
        return False
        
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if not msg:
        print("Error: Could not get global position for takeoff.")
        return False
        
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

def land_px4(master):
    """
    Land the drone.
    Sends MAV_CMD_NAV_LAND.
    """
    print("Sending LAND command...")
    
    # Command Land
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, # params 1-4 unused
        0, 0, 0     # params 5-7 (lat/lon/alt) - 0 for current position
    )
    
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Land ACK: {ack_msg}")
    
    if ack_msg and ack_msg.result == 0:
        print("Land command accepted.")
        return True
    else:
        print("Land command failed or no ACK received.")
        return False

def main():
    """Main function to connect, arm, and takeoff or land."""
    parser = argparse.ArgumentParser(
        description="Send takeoff or land commands via MAVLink UDP.",
        epilog="""Examples:
  %(prog)s --takeoff --altitude=15.5   # Takeoff to 15.5 meters
  %(prog)s --land --udp=14542           # Land command on port 14542
  %(prog)s --takeoff --altitude=20.0 --udp=14541""",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # Command group: Ensure at least one command is given
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--takeoff', action='store_true', help='Send takeoff command')
    group.add_argument('--land', action='store_true', help='Send land command')

    parser.add_argument('--altitude', type=float, default=10.0, help='Altitude for takeoff (default: 10.0m)')
    parser.add_argument('--udp', dest='udp_port', type=int, default=14541, help='UDP port (default: 14541)')

    args = parser.parse_args()

    print("=" * 60)
    print("MAVLink Command Sender")
    print("=" * 60)

    try:
        # Connect to drone
        master = connect_to_drone(args.udp_port)
        
        # Small delay to ensure connection is stable
        time.sleep(1)
        
        if args.land:
            print("Command: LAND")
            if not land_px4(master):
                print("Landing sequence failed or timed out.")
                sys.exit(1)
            print("\nLanding command sent successfully!")
            print("Monitor the drone's status to confirm landing.")
            
        else:
            print(f"Command: TAKEOFF to {args.altitude}m")
            if not takeoff_px4(master, takeoff_altitude=args.altitude):
                print("Takeoff sequence failed. Exiting.")
                sys.exit(1)
            
            print("\nTakeoff sequence completed successfully!")
            print("Monitor the drone's status to confirm takeoff.")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()
