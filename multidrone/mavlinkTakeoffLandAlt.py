#!/usr/bin/env python3
"""
Simple script to connect to a drone via UDP and send arm and takeoff commands.
Uses pymavlink library to communicate with the drone on UDP port (default: 14541).
"""

import sys
import time
import argparse
import math
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

def verify_takeoff(master):
    """
    Verify if the drone has successfully taken off by checking EXTENDED_SYS_STATE.
    Returns True if drone is in air, False otherwise.
    """
    ext_state_msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=5)
    if ext_state_msg:
        landed_state = ext_state_msg.landed_state
        if landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
            return True
        elif landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF:
            # Still in takeoff phase, consider it as in progress
            return True
    return False

def takeoff_px4(master, takeoff_altitude=10.0, max_attempts=3):
    """
    Takeoff function with verification and retry logic.
    Order: Mode change -> Takeoff command -> Arm
    Verifies takeoff after 15 seconds and retries up to max_attempts times.
    """
    print("Connected to PX4 autopilot")
    
    for attempt in range(1, max_attempts + 1):
        print(f"\n{'='*60}")
        print(f"Takeoff Attempt {attempt}/{max_attempts}")
        print(f"{'='*60}")
        
        try:
            mode_id = master.mode_mapping()["TAKEOFF"][1]
        except KeyError:
            print("Error: TAKEOFF mode not supported by this autopilot.")
            return False
            
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            print("Error: Could not get global position for takeoff.")
            if attempt < max_attempts:
                print("Retrying...")
                time.sleep(2)
                continue
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
        
        # Wait 15 seconds before verification
        print("Waiting 15 seconds before verification...")
        time.sleep(15)
        
        # Verify takeoff
        print("Verifying takeoff status...")
        if verify_takeoff(master):
            print(f"✓ Takeoff successful on attempt {attempt}!")
            return True
        else:
            print(f"✗ Takeoff verification failed on attempt {attempt}")
            if attempt < max_attempts:
                print("Drone is not in the air. Retrying...")
                time.sleep(2)
            else:
                print(f"\n{'='*60}")
                print(f"TAKEOFF FAILED after {max_attempts} attempts")
                print(f"{'='*60}")
                return False
    
    return False

def verify_landing(master):
    """
    Verify if the drone has successfully landed by checking EXTENDED_SYS_STATE.
    Returns True if drone is on ground, False otherwise.
    """
    ext_state_msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=5)
    if ext_state_msg:
        landed_state = ext_state_msg.landed_state
        if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            return True
    return False

def land_px4(master, max_attempts=3):
    """
    Land the drone with verification and retry logic.
    Sends MAV_CMD_NAV_LAND and verifies landing after 15 seconds.
    Retries up to max_attempts times if landing fails.
    """
    
    for attempt in range(1, max_attempts + 1):
        print(f"\n{'='*60}")
        print(f"Landing Attempt {attempt}/{max_attempts}")
        print(f"{'='*60}")
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
        
        if ack_msg and ack_msg.result != 0:
            print(f"✗ Land command rejected (result={ack_msg.result})")
            if attempt < max_attempts:
                print("Retrying...")
                time.sleep(2)
                continue
            else:
                print(f"\n{'='*60}")
                print(f"LANDING FAILED after {max_attempts} attempts")
                print(f"{'='*60}")
                return False
        
        # Wait 15 seconds before verification
        print("Waiting 15 seconds before verification...")
        time.sleep(15)
        
        # Verify landing
        print("Verifying landing status...")
        if verify_landing(master):
            print(f"✓ Landing successful on attempt {attempt}!")
            return True
        else:
            print(f"✗ Landing verification failed on attempt {attempt}")
            if attempt < max_attempts:
                print("Drone is not on the ground. Retrying...")
                time.sleep(2)
            else:
                print(f"\n{'='*60}")
                print(f"LANDING FAILED after {max_attempts} attempts")
                print(f"{'='*60}")
                return False
    
    return False

def set_altitude_px4(master, altitude, max_attempts=3):
    """
    Set the drone's altitude using SET_POSITION_TARGET_LOCAL_NED in OFFBOARD mode.
    Uses real-time feedback loop to monitor altitude and exit when target is reached.
    """
    threshold = 0.5  # Altitude threshold in meters
    timeout = 30.0   # Maximum time to wait for altitude change
    
    for attempt in range(1, max_attempts + 1):
        print(f"\n{'='*60}")
        print(f"Set Altitude Attempt {attempt}/{max_attempts}")
        print(f"{'='*60}")
        print(f"Applying SET_ALTITUDE: {altitude}m...")
        
        # 1. Check if drone is armed
        hb_msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if hb_msg:
            armed = bool(hb_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print("Warning: Drone is not armed. Altitude change may fail.")
        
        # 2. Fetch current local position
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        if not msg:
            print("Error: Could not get local position.")
            if attempt < max_attempts:
                print("Retrying...")
                time.sleep(2)
                continue
            return False
        
        current_x, current_y = msg.x, msg.y
        current_alt = -msg.z
        target_z = -float(altitude)
        altitude_error = abs(current_alt - altitude)
        
        print(f"Current: X={current_x:.2f}, Y={current_y:.2f}, Z={msg.z:.2f} (alt={current_alt:.2f}m)")
        print(f"Target: Z={target_z:.2f} (alt={altitude:.2f}m)")
        
        # EARLY EXIT: Check if already at altitude
        if altitude_error < threshold:
             print(f"✓ Already at target altitude ({current_alt:.2f}m).")
             return True

        # 3. Send initial setpoints (required before OFFBOARD) - increased from 10 to 20
        print("Sending pre-OFFBOARD setpoints...")
        for _ in range(20):
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0xDF8,
                current_x, current_y, target_z,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)  # 10Hz
        
        # 4. Switch to OFFBOARD mode
        print("Switching to OFFBOARD mode...")
        master.set_mode('OFFBOARD')
        time.sleep(0.5)
        
        # Verify mode switch
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            mode_name = master.flightmode if hasattr(master, 'flightmode') else 'UNKNOWN'
            print(f"  Current mode: {mode_name}")
        
        # 5. Control loop with real-time altitude monitoring
        print(f"Monitoring altitude (timeout: {timeout}s, threshold: ±{threshold}m)...")
        start_time = time.time()
        last_status_time = start_time
        reached = False
        loop_count = 0
        
        while time.time() - start_time < timeout:
            # Send position setpoint at 10Hz
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0xDF8,
                current_x, current_y, target_z,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            loop_count += 1
            
            # Check current altitude (blocking wait for up to 0.1s ensures we get the message)
            pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
            if pos:
                current_alt = -pos.z
                altitude_error = abs(current_alt - altitude)
                
                # Print status every 2 seconds
                if time.time() - last_status_time >= 2.0:
                    elapsed = time.time() - start_time
                    print(f"  [{elapsed:.1f}s] Alt: {current_alt:.2f}m, Target: {altitude:.2f}m, Error: {altitude_error:.2f}m")
                    last_status_time = time.time()
                
                # Check if target reached
                if altitude_error < threshold:
                    print(f"✓ Altitude reached: {current_alt:.2f}m (target: {altitude:.2f}m, error: {altitude_error:.2f}m)")
                    reached = True
                    break
            
            # Note: sleep is implicit in recv_match(timeout=0.1) if NO message arrives.
            # If message arrives instantly, we loop faster, which is fine (more setpoints).
            # To strictly limit setpoint rate to ~10Hz, we could add small sleep if recv was fast.
            # But standard practice is sending setpoints as fast as state updates or slightly faster.
            # Checking elapsed time to enforce 10Hz minimum interval is optional but robust.
            
        
        if not reached:
            # Final altitude check
            pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
            if pos:
                current_alt = -pos.z
                altitude_error = abs(current_alt - altitude)
                print(f"Timeout after {timeout}s. Final altitude: {current_alt:.2f}m (error: {altitude_error:.2f}m)")
        
        # 6. Switch back to HOLD mode
        print("Switching to HOLD mode...")
        mode = 'LOITER' if 'LOITER' in master.mode_mapping() else 'HOLD'
        master.set_mode(mode)
        time.sleep(0.5)
        
        # 7. Final verification
        if reached:
             # Already verified in loop
             print(f"✓ Altitude command successful on attempt {attempt}!")
             return True

        print("Final altitude verification...")
        time.sleep(1)
        pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        if pos:
            current_alt = -pos.z
            altitude_error = abs(current_alt - altitude)
            print(f"Final altitude: {current_alt:.2f}m (target: {altitude}m, error: {altitude_error:.2f}m)")
            
            if altitude_error < threshold:
                print(f"✓ Altitude command successful on attempt {attempt}!")
                return True
            else:
                print(f"✗ Altitude verification failed on attempt {attempt}")
                if attempt < max_attempts:
                    print(f"Altitude not reached (error: {altitude_error:.2f}m). Retrying...")
                    time.sleep(2)
                else:
                    print(f"\n{'='*60}")
                    print(f"SET ALTITUDE FAILED after {max_attempts} attempts")
                    print(f"{'='*60}")
                    return False
        else:
            print("Error: Could not verify altitude (no position data)")
            if attempt < max_attempts:
                print("Retrying...")
                time.sleep(2)
            else:
                print(f"\n{'='*60}")
                print(f"SET ALTITUDE FAILED after {max_attempts} attempts")
                print(f"{'='*60}")
                return False
    
    return False

def get_drone_state(master):
    """
    Get the drone's current state including position, armed status, and flight status.
    Returns a dictionary with state information.
    """
    state = {
        'x': None,
        'y': None,
        'z': None,
        'armed': False,
        'landed_state': 'UNKNOWN'
    }
    
    # Get local position (NED coordinates)
    pos_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    if pos_msg:
        state['x'] = pos_msg.x
        state['y'] = pos_msg.y
        state['z'] = pos_msg.z
    else:
        print("Warning: Could not get LOCAL_POSITION_NED")
    
    # Get armed status from heartbeat
    hb_msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if hb_msg:
        # Check if MAV_MODE_FLAG_SAFETY_ARMED bit is set
        state['armed'] = bool(hb_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    else:
        print("Warning: Could not get HEARTBEAT")
    
    # Get landed state
    ext_state_msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=5)
    if ext_state_msg:
        landed_state = ext_state_msg.landed_state
        if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            state['landed_state'] = 'ON_GROUND'
        elif landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
            state['landed_state'] = 'IN_AIR'
        elif landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF:
            state['landed_state'] = 'TAKEOFF'
        elif landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
            state['landed_state'] = 'LANDING'
        else:
            state['landed_state'] = 'UNDEFINED'
    else:
        print("Warning: Could not get EXTENDED_SYS_STATE")
    
    return state

def main():
    """Main function to connect, arm, and takeoff or land."""
    parser = argparse.ArgumentParser(
        description="Send takeoff, land, set altitude, or query state commands via MAVLink UDP.",
        epilog="""Examples:
  %(prog)s --takeoff --altitude=15.5   # Takeoff to 15.5 meters
  %(prog)s --land --udp=14542           # Land command on port 14542
  %(prog)s --altitude=10.0 --udp=14541  # Set altitude to 10.0m
  %(prog)s --state --udp=14541          # Query drone state""",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # Command group: Make optional to allow altitude-only commands
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('--takeoff', action='store_true', help='Send takeoff command')
    group.add_argument('--land', action='store_true', help='Send land command')
    group.add_argument('--state', action='store_true', help='Query drone state (position, armed, landed)')

    parser.add_argument('--altitude', type=float, default=10.0, metavar='ALTITUDE', help='Altitude for takeoff/set (default: 10.0m)')
    parser.add_argument('--udp', dest='udp_port', type=int, default=14541, metavar='PORT', help='UDP port (default: 14541)')

    # Strict argument pattern enforcement (--name=value)
    for arg in sys.argv[1:]:
        if arg.startswith('--altitude') and '=' not in arg and len(arg) == 10:
             print("Error: --altitude must be passed as --altitude=VALUE")
             sys.exit(1)
        if arg.startswith('--udp') and '=' not in arg and len(arg) == 5:
             print("Error: --udp must be passed as --udp=VALUE")
             sys.exit(1)

    args = parser.parse_args()

    # Determine command
    altitude_provided = any(arg.startswith('--altitude') for arg in sys.argv)
    
    if not (args.takeoff or args.land or args.state or altitude_provided):
        parser.print_help()
        sys.exit(0)

    print("=" * 60)
    print("MAVLink Command Sender")
    print("=" * 60)

    try:
        # Connect to drone
        master = connect_to_drone(args.udp_port)
        
        # Small delay to ensure connection is stable
        time.sleep(1)
        
        if args.state:
            print("Command: QUERY STATE")
            state = get_drone_state(master)
            
            print("\n" + "=" * 60)
            print("DRONE STATE")
            print("=" * 60)
            print(f"Position (Local NED):")
            print(f"  X: {state['x']:.3f} m" if state['x'] is not None else "  X: N/A")
            print(f"  Y: {state['y']:.3f} m" if state['y'] is not None else "  Y: N/A")
            print(f"  Z: {state['z']:.3f} m" if state['z'] is not None else "  Z: N/A")
            print(f"Armed: {state['armed']}")
            print(f"Flight Status: {state['landed_state']}")
            print("=" * 60)
            
        elif args.land:
            print("Command: LAND")
            if not land_px4(master):
                print("Landing sequence failed or timed out.")
                sys.exit(1)
            print("\nLanding command sent successfully!")
            print("Monitor the drone's status to confirm landing.")
            
        elif args.takeoff:
            print(f"Command: TAKEOFF to {args.altitude}m")
            if not takeoff_px4(master, takeoff_altitude=args.altitude):
                print("Takeoff sequence failed. Exiting.")
                sys.exit(1)
            
            print("\nTakeoff sequence completed successfully!")
            print("Monitor the drone's status to confirm takeoff.")
        
        else:
            # Altitude only
            print(f"Command: SET_ALTITUDE to {args.altitude}m")
            if not set_altitude_px4(master, altitude=args.altitude):
                print("Set altitude command failed. Exiting.")
                sys.exit(1)
            print("\nSet altitude command sent successfully!")
        
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
