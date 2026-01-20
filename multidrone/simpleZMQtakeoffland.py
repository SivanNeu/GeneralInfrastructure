#!/usr/bin/env python3
"""
Simple script to send a takeoff or land command to the drone via ZMQ.
Connects to the hardware_adapter and sends:
 - Takeoff command with specified altitude.
 - Land command.
"""

import zmq
import struct
import time
import sys
import argparse

# ZMQ configuration
ZMQ_CMD_PORT_DEFAULT = 7793
TAKEOFF_TOPIC = b'quadTakeoffCmd'
LAND_TOPIC = b'quadLandCmd'

def send_command(topic, data, topic_name="Command", zmq_port=ZMQ_CMD_PORT_DEFAULT):
    """
    Generic function to send a command via ZMQ.
    """
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.setsockopt(zmq.LINGER, 0)

    try:
        bind_addr = f"tcp://*:{zmq_port}"
        pub_socket.bind(bind_addr)
        print(f"ZMQ Publisher: Bound to {bind_addr}")
        
        print("Waiting 2 seconds for subscribers (hardware_adapter) to connect...")
        time.sleep(2.0)
        
        print(f"Sending {topic_name}:")
        print(f"  Topic: {topic}")
        print(f"  Data (hex): {data.hex()}")
        
        # Concatenate topic and data into a single frame to support ZMQ_CONFLATE
        pub_socket.send(topic + data)
        print(f"✓ {topic_name} sent successfully!")
        
        # Redundancy
        print(f"Sending {topic_name} 2 more times...")
        for i in range(2):
            time.sleep(0.5)
            pub_socket.send(topic + data)
        
        time.sleep(0.2)
        return True

    except Exception as e:
        print(f"✗ Error sending {topic_name}: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        pub_socket.close()
        context.term()

def send_takeoff_command(altitude=10.0, zmq_port=ZMQ_CMD_PORT_DEFAULT):
    altitude_data = struct.pack('d', altitude)
    return send_command(TAKEOFF_TOPIC, altitude_data, "Takeoff Command", zmq_port)

def send_land_command(zmq_port=ZMQ_CMD_PORT_DEFAULT):
    # Sending 0.0 as payload for land, just to have a valid double payload if expected
    data = struct.pack('d', 0.0)
    return send_command(LAND_TOPIC, data, "Land Command", zmq_port)

def main():
    parser = argparse.ArgumentParser(
        description="Send takeoff or land commands via ZMQ.",
        epilog="""Examples:
  %(prog)s --takeoff --altitude=15.5   # Takeoff to 15.5 meters
  %(prog)s --land --zmq=8800           # Land command on port 8800
  %(prog)s --takeoff --altitude=20.0 --zmq=7794""",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # Command group: Ensure at least one command is given
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--takeoff', action='store_true', help='Send takeoff command')
    group.add_argument('--land', action='store_true', help='Send land command')

    parser.add_argument('--altitude', type=float, default=10.0, help='Altitude for takeoff (default: 10.0m)')
    parser.add_argument('--zmq', dest='zmq_port', type=int, default=ZMQ_CMD_PORT_DEFAULT, help=f'ZMQ port (default: {ZMQ_CMD_PORT_DEFAULT})')

    args = parser.parse_args()

    print("=" * 60)
    print("ZMQ Command Sender")
    print("=" * 60)

    success = False
    if args.takeoff:
        print(f"Command: TAKEOFF to {args.altitude}m")
        success = send_takeoff_command(args.altitude, args.zmq_port)
    elif args.land:
        print("Command: LAND")
        success = send_land_command(args.zmq_port)

    if success:
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
