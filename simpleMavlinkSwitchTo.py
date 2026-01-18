#!/usr/bin/env python3
"""
Simple script to switch drone state between different modes via MAVLink.
Connects to the drone via UDP and sends mode change commands.
"""

import sys
import time
from pymavlink import mavutil

# PX4 mode definitions
# Format: {mode_name: (px4_mode_name_in_mapping, custom_mode_number, expected_custom_mode_value)}
# Note: HOLD mode in PX4 is often called LOITER in the mode mapping
PX4_MODES = {
    'HOLD': ('LOITER', 4, None),      # HOLD mode maps to LOITER in PX4, custom_mode=4
    'LOITER': ('LOITER', 4, None),    # LOITER mode (same as HOLD)
    'OFFBOARD': ('OFFBOARD', 6, 393216),    # OFFBOARD mode: custom_mode=6, expected=0x60000
    # Add more modes here in the future
    # 'MANUAL': ('MANUAL', 1, ...),
    # 'ALTCTL': ('ALTCTL', 2, ...),
    # 'POSCTL': ('POSCTL', 3, ...),
    # 'AUTO': ('MISSION', 5, ...),
    # 'ACRO': ('ACRO', 7, ...),
    # 'STABILIZED': ('STABILIZED', 8, ...),
    # 'RATTITUDE': ('RATTITUDE', 9, ...),
}

UDP_PORT_DEFAULT = 14540

def print_help():
    """Print help message and exit."""
    script_name = sys.argv[0] if sys.argv else "simpleMavlinkSwitchTo.py"
    modes_list = ', '.join(PX4_MODES.keys())
    print(f"""
Usage: {script_name} [OPTIONS]

Switch drone state between different modes via MAVLink.

Options:
  -h, --help            Show this help message and exit
  --state=STATE         Desired state/mode name (required)
                        Available states: {modes_list}
  --udp=PORT            UDP port to connect to (default: {UDP_PORT_DEFAULT})

Examples:
  {script_name} --state=HOLD --udp=14540      # Switch to HOLD mode on port 14540
  {script_name} --state=OFFBOARD              # Switch to OFFBOARD mode on default port
  {script_name} --help                        # Show this help message

Notes:
  - The script connects to the drone via UDP
  - Sends MAV_CMD_DO_SET_MODE command to change mode
  - Waits for COMMAND_ACK and HEARTBEAT confirmation
  - Returns exit code 0 on success, 1 on failure
""")
    sys.exit(0)

def connect_to_drone(udp_port):
    """Connect to the drone via UDP."""
    udp_address = f"127.0.0.1:{udp_port}"
    print(f"Connecting to drone on UDP {udp_address}...")
    try:
        master = mavutil.mavlink_connection(f"udp:{udp_address}")
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=5.0)
        print(f"✓ Heartbeat received! System ID: {master.target_system}, Component ID: {master.target_component}")
        return master
    except Exception as e:
        print(f"✗ Error connecting to drone: {e}")
        raise

def switch_mode(master, mode_name):
    """
    Switch drone to the specified mode.
    
    Args:
        master: MAVLink connection object
        mode_name: Name of the mode (must be in PX4_MODES)
    
    Returns:
        True if successful, False otherwise
    """
    if mode_name.upper() not in PX4_MODES:
        print(f"✗ Error: Unknown mode '{mode_name}'")
        print(f"  Available modes: {', '.join(PX4_MODES.keys())}")
        return False
    
    mode_name_upper = mode_name.upper()
    px4_mode_name, custom_mode_num, expected_custom_mode = PX4_MODES[mode_name_upper]
    
    # Use target system/component from master
    ts = master.target_system
    tc = master.target_component
    
    if ts == 0:
        print("✗ Error: No target system ID available. Make sure drone is connected.")
        return False
    
    print(f"Switching to {mode_name_upper} mode (target: system={ts}, component={tc})...")
    if px4_mode_name != mode_name_upper:
        print(f"  Note: {mode_name_upper} maps to {px4_mode_name} in PX4")
    
    # Try method 1: Use master.set_mode() if available (most reliable)
    try:
        mode_mapping = master.mode_mapping()
        print(f"  Available modes in mapping: {list(mode_mapping.keys())}")
        
        if px4_mode_name in mode_mapping:
            print(f"  Using master.set_mode() method with '{px4_mode_name}'...")
            master.set_mode(px4_mode_name)
            print(f"  Mode change command sent via set_mode()")
            
            # Get the expected custom_mode from the mapping
            if expected_custom_mode is None:
                try:
                    mode_info = mode_mapping[px4_mode_name]
                    if len(mode_info) >= 2:
                        # Calculate expected custom_mode (mode_info[1] is the mode number)
                        # PX4 custom_mode format: main_mode << 16 | sub_mode
                        # For most modes: main_mode is the mode number, sub_mode is usually 0
                        mode_num = mode_info[1]
                        # PX4 uses: (main_mode << 16) | sub_mode
                        # For simple modes, sub_mode is often 0, so: mode_num << 16
                        # But we need to check the actual format
                        # Let's get the current mode to understand the format better
                        expected_custom_mode = mode_num << 16  # This might need adjustment
                except:
                    pass
        else:
            # Mode not in mapping, try using command_long with mode_mapping lookup
            print(f"  Mode '{px4_mode_name}' not in mode_mapping, trying alternative approach...")
            raise ValueError("Mode not in mapping")
    except (AttributeError, ValueError, KeyError) as e:
        # Fallback: Try SET_MODE message directly (sometimes more reliable for PX4)
        print(f"  Trying SET_MODE message directly...")
        
        # Try to get mode ID from mode_mapping
        mode_id = None
        try:
            mode_mapping = master.mode_mapping()
            if px4_mode_name in mode_mapping:
                mode_id = mode_mapping[px4_mode_name][1]
                print(f"    Mode ID from mapping for '{px4_mode_name}': {mode_id}")
            else:
                print(f"    Mode '{px4_mode_name}' not found in mode_mapping, using custom_mode={custom_mode_num}")
        except Exception as e2:
            print(f"    Could not get mode_mapping: {e2}")
            print(f"    Using custom_mode={custom_mode_num}")
        
        # Use mode_id from mapping if available, otherwise use custom_mode_num
        mode_value = int(mode_id if mode_id is not None else custom_mode_num)
        
        # If we got mode_id from mapping, update expected_custom_mode
        if mode_id is not None and expected_custom_mode is None:
            # PX4 custom_mode format: (main_mode << 16) | sub_mode
            # For most modes, sub_mode is 0, so: mode_id << 16
            expected_custom_mode = mode_id << 16
            print(f"    Calculated expected custom_mode: {expected_custom_mode}")
        
        # Send SET_MODE message directly (sometimes more reliable than COMMAND_LONG)
        print(f"  Sending SET_MODE message (base_mode=1, custom_mode={mode_value})...")
        for i in range(5):
            master.mav.set_mode_send(
                ts,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
                mode_value  # custom_mode
            )
            time.sleep(0.1)
        
        # Also try COMMAND_LONG as backup
        print(f"  Also sending COMMAND_LONG as backup...")
        for i in range(3):
            master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # 176
                0,  # confirmation
                1.0,  # param1: base_mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1)
                float(mode_value),  # param2: custom_mode
                0.0,  # param3
                0.0,  # param4
                0.0,  # param5
                0.0,  # param6
                0.0,  # param7
            )
            time.sleep(0.1)
    
    # Small delay to let messages propagate
    time.sleep(0.2)
    
    # Wait for COMMAND_ACK
    print("Waiting for COMMAND_ACK...")
    start_time = time.time()
    timeout = 5.0  # Increased timeout
    ack_received = False
    
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.5)
        if msg is not None:
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                ack_received = True
                try:
                    result_name = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
                except:
                    result_name = f"UNKNOWN({msg.result})"
                
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"✓ COMMAND_ACK received: {result_name}")
                else:
                    print(f"✗ COMMAND_ACK received but command was not accepted: {result_name}")
                    return False
                break
        time.sleep(0.1)
    
    if not ack_received:
        print("⚠ Warning: No COMMAND_ACK received within timeout")
        print("  This may be normal for some autopilots - continuing to check HEARTBEAT...")
    
    # Wait and check for mode change confirmation via heartbeat
    print(f"Waiting for {mode_name_upper} mode confirmation via HEARTBEAT...")
    start_time = time.time()
    timeout = 8.0  # Increased timeout for mode change
    mode_confirmed = False
    last_heartbeat_time = start_time
    last_print_time = start_time
    
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.5)
        if msg is not None:
            last_heartbeat_time = time.time()
            current_mode = msg.custom_mode
            
            # Try to get mode name from master if available
            try:
                mode_name_current = master.flightmode
            except:
                mode_name_current = "UNKNOWN"
            
            # Check if mode matches expected value, or if mode name matches
            mode_matches = False
            if expected_custom_mode is not None:
                mode_matches = (current_mode == expected_custom_mode)
            else:
                # If we don't have expected_custom_mode, check by mode name
                mode_matches = (mode_name_current.upper() == px4_mode_name.upper())
            
            if mode_matches or mode_name_current.upper() == px4_mode_name.upper():
                print(f"✓ {mode_name_upper} mode confirmed! (custom_mode={current_mode}, mode_name={mode_name_current})")
                mode_confirmed = True
                break
            else:
                # Only print every 0.5 seconds to reduce spam
                if time.time() - last_print_time >= 0.5:
                    expected_str = f"{expected_custom_mode}" if expected_custom_mode is not None else f"mode_name={px4_mode_name}"
                    print(f"  Current mode: {mode_name_current} (custom_mode={current_mode}, waiting for {expected_str})...")
                    last_print_time = time.time()
        else:
            # Check if we're still receiving heartbeats
            if time.time() - last_heartbeat_time > 2.0:
                print("  ⚠ Warning: No HEARTBEAT received recently, connection may be lost")
                last_heartbeat_time = time.time()  # Reset to avoid spam
        
        time.sleep(0.1)
    
    if not mode_confirmed:
        print(f"✗ {mode_name_upper} mode not confirmed after {timeout} seconds")
        print("  The autopilot may not have received the command or may require additional setup.")
        if expected_custom_mode is not None:
            print(f"  Expected custom_mode: {expected_custom_mode}")
        else:
            print(f"  Expected mode name: {px4_mode_name}")
        return False
    
    return True

def main():
    """Main function."""
    # Check for help option first
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print_help()
    
    # Parse command line arguments
    udp_port = UDP_PORT_DEFAULT
    desired_state = None
    
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        
        # Check for help
        if arg in ['-h', '--help']:
            print_help()
        
        # Check for --state option
        elif arg.startswith('--state='):
            try:
                desired_state = arg.split('=', 1)[1]
            except IndexError:
                print("Error: --state requires a value")
                print("  Use --state=STATE_NAME (e.g., --state=HOLD)")
                print("  Use --help for more information")
                sys.exit(1)
        
        elif arg == '--state':
            if i + 1 < len(args):
                desired_state = args[i + 1]
                i += 1  # Skip the next argument
            else:
                print("Error: --state requires a value")
                print("  Use --state=STATE_NAME or --state STATE_NAME")
                print("  Use --help for more information")
                sys.exit(1)
        
        # Check for --udp option
        elif arg.startswith('--udp='):
            try:
                udp_port = int(arg.split('=', 1)[1])
            except (ValueError, IndexError):
                print(f"Error: Invalid UDP port value: {arg}")
                print("  Use --udp=PORT where PORT is an integer")
                print("  Use --help for more information")
                sys.exit(1)
        
        elif arg == '--udp':
            if i + 1 < len(args):
                try:
                    udp_port = int(args[i + 1])
                    i += 1  # Skip the next argument
                except ValueError:
                    print(f"Error: Invalid UDP port value: {args[i + 1]}")
                    print("  Use --udp PORT where PORT is an integer")
                    print("  Use --help for more information")
                    sys.exit(1)
            else:
                print("Error: --udp requires a port number")
                print("  Use --udp=PORT or --udp PORT")
                print("  Use --help for more information")
                sys.exit(1)
        
        else:
            print(f"Error: Unknown option: {arg}")
            print("  Use --help for more information")
            sys.exit(1)
        
        i += 1
    
    # Validate required arguments
    if desired_state is None:
        print("Error: --state is required")
        print("  Use --state=STATE_NAME (e.g., --state=HOLD)")
        print("  Use --help for more information")
        sys.exit(1)
    
    # Validate port range
    if udp_port < 1 or udp_port > 65535:
        print(f"Error: Invalid UDP port: {udp_port}")
        print("  Port must be between 1 and 65535")
        sys.exit(1)
    
    # Validate state
    if desired_state.upper() not in PX4_MODES:
        print(f"Error: Unknown state '{desired_state}'")
        print(f"  Available states: {', '.join(PX4_MODES.keys())}")
        print("  Use --help for more information")
        sys.exit(1)
    
    # Execute mode switch
    print("=" * 60)
    print("MAVLink Mode Switch")
    print("=" * 60)
    print(f"Target state: {desired_state.upper()}")
    print(f"UDP port: {udp_port}")
    print("")
    
    try:
        # Connect to drone
        master = connect_to_drone(udp_port)
        
        # Small delay to ensure connection is stable
        time.sleep(0.5)
        
        # Switch mode
        success = switch_mode(master, desired_state)
        
        if success:
            print("\n" + "=" * 60)
            print("✓ Mode switch completed successfully!")
            print("=" * 60)
            sys.exit(0)
        else:
            print("\n" + "=" * 60)
            print("✗ Mode switch failed!")
            print("=" * 60)
            sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting.")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
