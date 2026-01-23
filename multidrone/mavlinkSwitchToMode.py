#!/usr/bin/env python3
"""
Script to switch drone flight mode via MAVLink UDP.
Usage: ./mavlinkSwitchToMode.py --udp=PORT --mode=MODE
Example: ./mavlinkSwitchToMode.py --udp=14541 --mode=OFFBOARD
"""

import sys
import time
import argparse
from pymavlink import mavutil

# Mapping of common commercial flight modes to PX4 custom mode IDs
# PX4 Custom Mode IDs (auto-generated from PX4 firmware source if needed, but standard ones are known)
# Reference: https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/commander/px4_custom_mode.h
# For basic modes, we uses pymavlink's mode_mapping()
# Common PX4 modes: MANUAL, STABILIZED, ACRO, ALTCTL, POSCTL, OFFBOARD, MISSION, LOITER (HOLD), RTL, LAND, TAKEOFF

# PX4 Mode mapping provided by user
# These are the numerical values of custom_mode in HEARTBEAT messages
PX4_MODE_MAP = {
    'OFFBOARD': 393216,
    'HOLD': 50593792,
    'RETURN': 84148224,
    'TAKEOFF': 33816576,
    'LAND': 100925440,
    'GENERAL': 65535,
    'AUTO': 262144,
    'ACRO': 327680,
    'RATTITUDE': 524288,
    'ALTITUDE': 131072,
    'POSITION': 196608,
    'LOITER': 262147,
    'MISSION': 262148,
    'MANUAL': 65536,
    'STABILIZED': 458752,
    'POSITION_SLOW': 33751040,
    'SAFE_RECOVERY': 84148224,
    'FOLLOW_TARGET': 134479872,
    'PRECISION_LAND': 151257088,
    'MANUAL_0': 0  # Raw mode 0 often corresponds to MANUAL or uninitialized state
}

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

def switch_mode(master, mode_name, max_attempts=3):
    """
    Switch the drone to the specified mode.
    """
    # Check if mode exists in mapping
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        print(f"Error: Mode '{mode_name}' not supported by this autopilot.")
        print(f"Available modes: {list(mode_map.keys())}")
        return False
        
    mode_id = mode_map[mode_name]
    print(f"Attempting to switch to mode: {mode_name} (ID: {mode_id})")
    
    for attempt in range(1, max_attempts + 1):
        print(f"\nAttempt {attempt}/{max_attempts}")
        
        # Send mode switch command
        # For PX4, we use set_mode() helper from pymavlink
        # It's better to pass the name string; pymavlink handles the tuple mapping internally.
        master.set_mode(mode_name)
        
        # Wait for ACK
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack:
            print(f"Mode Switch ACK: {ack}")
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                # Command accepted, now verify actual state change
                if verify_mode(master, mode_name):
                    print(f"✓ Successfully switched to {mode_name}")
                    return True
            else:
                print(f"✗ Mode switch command rejected/failed (Result: {ack.result})")
        else:
            print("Warning: No ACK received for mode switch command")
            
        # Fallback verification in case ACK was lost or delayed
        if verify_mode(master, mode_name):
             print(f"✓ Successfully switched to {mode_name} (verified via heartbeat)")
             return True
             
        time.sleep(1)
        
    print(f"Failed to switch to {mode_name} after {max_attempts} attempts")
    return False

def verify_mode(master, target_mode, timeout=5):
    """
    Verify if the drone is in the target mode by monitoring heartbeats.
    """
    if target_mode not in PX4_MODE_MAP:
        # If not in our manual map, fallback to string comparison
        print(f"Warning: '{target_mode}' not in manual mode map, using heartbeat string comparison")
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            # Pymavlink helper to interpret custom_mode
            current_custom_mode = msg.custom_mode
            
            # Check against manual map if available
            if target_mode in PX4_MODE_MAP:
                if current_custom_mode == PX4_MODE_MAP[target_mode]:
                    return True
            
            # Fallback to string comparison via pymavlink's flightmode attribute
            if hasattr(master, 'flightmode') and master.flightmode == target_mode:
                return True
                
    return False

def main():
    parser = argparse.ArgumentParser(
        description="Switch drone flight mode or query current status via MAVLink UDP.",
        epilog="""Examples:
  %(prog)s --udp=14541 --mode=OFFBOARD      # Switch to OFFBOARD
  %(prog)s --udp=14541 --mode=HOLD          # Switch to HOLD (LOITER)
  %(prog)s --udp=14541 --status             # Query current mode""",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    parser.add_argument('--udp', dest='udp_port', type=int, required=True, metavar='PORT', help='UDP port (e.g., 14541)')
    parser.add_argument('--status', action='store_true', help='Display current mode without switching')
    parser.add_argument('--mode', dest='mode', type=str, required=False, metavar='MODE', help='Target flight mode (e.g., OFFBOARD, HOLD, LOITER, POSCTL)')
    
    # Strict argument handling to enforce --name=value pattern if desired, 
    # but argparse handles space separation fine. We'll stick to standard argparse behavior.
    
    args = parser.parse_args()

    # Validate that either --mode or --status is provided
    if not args.mode and not args.status:
        parser.error("One of --mode or --status is required.")
    
    target_mode = None
    if args.mode:
        # Handle LOITER alias for HOLD if user requests HOLD but PX4 uses LOITER
        # In PX4, "HOLD" mode is usually "LOITER" or "AUTO.LOITER"
        # User asked for "HOLD (LOITTER)", suggesting they might use either term.
        # We will let pymavlink's map handle exact names, but offer a helpful alias.
        target_mode = args.mode.upper()
        
        # Simple alias handling
        if target_mode == 'HOLD':
            # Check if HOLD exists, if not try LOITER
            # Proper check happens after connection
            pass 

    print("=" * 60)
    print(f"MAVLink Mode Manager")
    print("=" * 60)
    
    try:
        master = connect_to_drone(args.udp_port)
        
        if args.status:
            # Just report status
            print("\nFetching current status...")
            # Wait for a heartbeat to ensure we have fresh data
            hb = master.wait_heartbeat()
            
            # Identify mode name using pymavlink's built-in flightmode attribute (most robust)
            mode_name = "UNKNOWN"
            if hasattr(master, 'flightmode'):
                mode_name = master.flightmode
            
            # Fallback to manual map if pymavlink failed or returned UNKNOWN
            if mode_name == "UNKNOWN":
                custom_mode = hb.custom_mode
                for name, val in PX4_MODE_MAP.items():
                    if val == custom_mode:
                        mode_name = name
                        break
                print(f"Current Mode: {mode_name} (Raw: {custom_mode})")
            else:
                print(f"Current Mode: {mode_name}")
            
            sys.exit(0)

        # Mode switching logic
        if target_mode:
            print(f"Target Mode: {target_mode}")
            # Resolve aliases if needed
            mode_map = master.mode_mapping()
            if target_mode == 'HOLD' and 'HOLD' not in mode_map and 'LOITER' in mode_map:
                print("Note: 'HOLD' mode not found, using 'LOITER' as equivalent.")
                target_mode = 'LOITER'
            
            if switch_mode(master, target_mode):
                print("\nOperation successful!")
                sys.exit(0)
            else:
                print("\nOperation failed.")
                sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
