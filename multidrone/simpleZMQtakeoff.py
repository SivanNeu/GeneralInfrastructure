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
ZMQ_CMD_PORT = 7793
TAKEOFF_TOPIC = b'quadTakeoffCmd'

def send_takeoff_command(altitude=10.0):
    """
    Send a takeoff command via ZMQ to the drone.
    
    Args:
        altitude: Desired takeoff altitude in meters (default: 10.0)
    """
    # Create ZMQ context and publisher socket
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.setsockopt(zmq.LINGER, 0)  # Close socket immediately on termination
    
    try:
        # Bind to the command port
        bind_addr = f"tcp://*:{ZMQ_CMD_PORT}"
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

def main():
    """Main function."""
    # Parse command line arguments
    altitude = 10.0  # Default altitude
    
    if len(sys.argv) > 1:
        try:
            altitude = float(sys.argv[1])
        except ValueError:
            print(f"Error: Invalid altitude value: {sys.argv[1]}")
            print("Usage: simpleZMQtakeoff.py [ALTITUDE]")
            print("  ALTITUDE    Takeoff altitude in meters (default: 10.0)")
            sys.exit(1)
    
    if len(sys.argv) > 2:
        print("Usage: simpleZMQtakeoff.py [ALTITUDE]")
        print("  ALTITUDE    Takeoff altitude in meters (default: 10.0)")
        sys.exit(1)
    
    # Send the takeoff command
    print("=" * 60)
    print("ZMQ Takeoff Command Sender")
    print("=" * 60)
    print(f"Port: {ZMQ_CMD_PORT}")
    print(f"Topic: {TAKEOFF_TOPIC.decode()}")
    print(f"Altitude: {altitude}m")
    print("")
    print("IMPORTANT: Make sure zmq_commands_mavlink is running!")
    print("  The hardware adapter must be subscribed to receive this command.")
    print("  Check for 'Hardware_adapter: Enqueued TAKEOFF command' in the adapter logs.")
    print("")
    
    if send_takeoff_command(altitude):
        print("\n" + "=" * 60)
        print("✓ Takeoff command sent successfully!")
        print("=" * 60)
        print("\nNext steps:")
        print("  1. Check hardware_adapter logs for 'Enqueued TAKEOFF command' message")
        print("  2. Check hardware_adapter logs for 'Starting takeoff sequence' message")
        print("  3. Monitor the drone's status to confirm takeoff")
        print("\nIf you don't see the messages in hardware_adapter logs:")
        print("  - Verify zmq_commands_mavlink is running")
        print("  - Verify it's subscribed to port 7793")
        print("  - Check for any error messages")
    else:
        print("\n✗ Failed to send takeoff command.")
        sys.exit(1)

if __name__ == "__main__":
    main()
