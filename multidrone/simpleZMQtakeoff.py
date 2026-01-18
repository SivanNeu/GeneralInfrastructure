#!/usr/bin/env python3
"""
Simple script to send a takeoff command to the drone via ZMQ.
Connects to the hardware_adapter and sends a takeoff command with specified altitude.
"""

import zmq
import struct
import time
import sys

# ZMQ configuration
ZMQ_CMD_PORT_DEFAULT = 7793
TAKEOFF_TOPIC = b'quadTakeoffCmd'

def send_takeoff_command(altitude=10.0, zmq_port=ZMQ_CMD_PORT_DEFAULT):
    """
    Send a takeoff command via ZMQ to the drone.
    
    Args:
        altitude: Desired takeoff altitude in meters (default: 10.0)
        zmq_port: ZMQ port to bind to (default: 7793)
    """
    # Create ZMQ context and publisher socket
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.setsockopt(zmq.LINGER, 0)  # Close socket immediately on termination
    
    try:
        # Bind to the command port
        bind_addr = f"tcp://*:{zmq_port}"
        pub_socket.bind(bind_addr)
        print(f"ZMQ Publisher: Bound to {bind_addr}")
        
        # Wait for subscribers to connect (important for ZMQ PUB/SUB pattern)
        # In ZMQ PUB/SUB, messages sent before subscribers connect are lost
        print("Waiting 2 seconds for subscribers (hardware_adapter) to connect...")
        print("  NOTE: Make sure zmq_commands_mavlink is running and subscribed!")
        time.sleep(2.0)
        
        # Send altitude as raw double (8 bytes) - simpler and more reliable than pickle
        # The C code handles both raw double and pickled formats, but raw double is more reliable
        altitude_data = struct.pack('d', altitude)  # 'd' = double (8 bytes)
        
        # Debug: Print what we're sending
        print(f"\nSending takeoff command:")
        print(f"  Topic: {TAKEOFF_TOPIC}")
        print(f"  Altitude: {altitude}m")
        print(f"  Data length: {len(altitude_data)} bytes (raw double)")
        print(f"  Data (hex): {altitude_data.hex()}")
        
        # Send as multipart message: [topic, data]
        pub_socket.send_multipart([TAKEOFF_TOPIC, altitude_data])
        print("✓ Takeoff command sent successfully!")
        
        # Send multiple times to ensure it's received (ZMQ PUB/SUB can drop messages)
        print("Sending takeoff command 2 more times to ensure delivery...")
        for i in range(2):
            time.sleep(0.5)
            pub_socket.send_multipart([TAKEOFF_TOPIC, altitude_data])
            print(f"  Sent attempt {i+2}/3")
        
        # Small delay to ensure messages are sent
        time.sleep(0.2)
        
    except Exception as e:
        print(f"✗ Error sending takeoff command: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Cleanup
        pub_socket.close()
        context.term()
    
    return True

def print_help():
    """Print help message and exit."""
    script_name = sys.argv[0] if sys.argv else "simpleZMQtakeoff.py"
    print(f"""
Usage: {script_name} [OPTIONS] [ALTITUDE]

Send a takeoff command to the drone via ZMQ.

Arguments:
  ALTITUDE              Takeoff altitude in meters (default: 10.0)

Options:
  -h, --help            Show this help message and exit
  --zmq=PORT            ZMQ port to bind to (default: {ZMQ_CMD_PORT_DEFAULT})

Examples:
  {script_name}                          # Takeoff to default altitude (10.0m) on default port ({ZMQ_CMD_PORT_DEFAULT})
  {script_name} 15.5                     # Takeoff to 15.5 meters
  {script_name} --zmq=8800               # Use ZMQ port 8800
  {script_name} --zmq=8800 15.5          # Use port 8800 and altitude 15.5m
  {script_name} --help                   # Show this help message

Configuration:
  Default ZMQ Port: {ZMQ_CMD_PORT_DEFAULT}
  Topic: {TAKEOFF_TOPIC.decode()}

Notes:
  - Make sure zmq_commands_mavlink is running before sending commands
  - The hardware adapter must be subscribed to the specified port to receive commands
  - Check hardware_adapter logs for confirmation of command receipt
""")
    sys.exit(0)

def main():
    """Main function."""
    # Default values
    altitude = 10.0  # Default altitude
    zmq_port = ZMQ_CMD_PORT_DEFAULT  # Default ZMQ port
    
    # Parse command line arguments
    args = sys.argv[1:]
    positional_args = []
    
    i = 0
    while i < len(args):
        arg = args[i]
        
        # Check for help
        if arg in ['-h', '--help']:
            print_help()
        
        # Check for --zmq option
        elif arg.startswith('--zmq='):
            # Format: --zmq=PORT
            try:
                zmq_port = int(arg.split('=', 1)[1])
            except (ValueError, IndexError):
                print(f"Error: Invalid ZMQ port value: {arg}")
                print("  Use --zmq=PORT where PORT is an integer")
                print("  Use --help for more information")
                sys.exit(1)
        
        elif arg == '--zmq':
            # Format: --zmq PORT
            if i + 1 < len(args):
                try:
                    zmq_port = int(args[i + 1])
                    i += 1  # Skip the next argument as it's the port value
                except ValueError:
                    print(f"Error: Invalid ZMQ port value: {args[i + 1]}")
                    print("  Use --zmq PORT where PORT is an integer")
                    print("  Use --help for more information")
                    sys.exit(1)
            else:
                print("Error: --zmq requires a port number")
                print("  Use --zmq=PORT or --zmq PORT")
                print("  Use --help for more information")
                sys.exit(1)
        
        else:
            # Positional argument (altitude)
            positional_args.append(arg)
        
        i += 1
    
    # Parse altitude from positional arguments
    if len(positional_args) > 0:
        try:
            altitude = float(positional_args[0])
        except ValueError:
            print(f"Error: Invalid altitude value: {positional_args[0]}")
            print("  ALTITUDE must be a number")
            print("  Use --help for more information")
            sys.exit(1)
    
    if len(positional_args) > 1:
        print("Error: Too many positional arguments")
        print("  Only one ALTITUDE value is allowed")
        print("  Use --help for more information")
        sys.exit(1)
    
    # Validate port range
    if zmq_port < 1 or zmq_port > 65535:
        print(f"Error: Invalid ZMQ port: {zmq_port}")
        print("  Port must be between 1 and 65535")
        sys.exit(1)
    
    # Send the takeoff command
    print("=" * 60)
    print("ZMQ Takeoff Command Sender")
    print("=" * 60)
    print(f"Port: {zmq_port}")
    print(f"Topic: {TAKEOFF_TOPIC.decode()}")
    print(f"Altitude: {altitude}m")
    print("")
    print("IMPORTANT: Make sure zmq_commands_mavlink is running!")
    print("  The hardware adapter must be subscribed to receive this command.")
    print("  Check for 'Hardware_adapter: Enqueued TAKEOFF command' in the adapter logs.")
    print("")
    
    if send_takeoff_command(altitude, zmq_port):
        print("\n" + "=" * 60)
        print("✓ Takeoff command sent successfully!")
        print("=" * 60)
        print("\nNext steps:")
        print("  1. Check hardware_adapter logs for 'Enqueued TAKEOFF command' message")
        print("  2. Check hardware_adapter logs for 'Starting takeoff sequence' message")
        print("  3. Monitor the drone's status to confirm takeoff")
        print(f"\nIf you don't see the messages in hardware_adapter logs:")
        print(f"  - Verify zmq_commands_mavlink is running")
        print(f"  - Verify it's subscribed to port {zmq_port}")
        print("  - Check for any error messages")
    else:
        print("\n✗ Failed to send takeoff command.")
        sys.exit(1)

if __name__ == "__main__":
    main()
