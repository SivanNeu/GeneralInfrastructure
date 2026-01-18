#!/usr/bin/env python3
"""
Simple ZMQ reader program that reads data from ZMQ port and prints it to screen.
Based on system_manager.py reference implementation.
"""
import sys

# Check for help option before importing other modules
if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
    script_name = sys.argv[0] if sys.argv else "simpleZMQread.py"
    print(f"""
Usage: {script_name} [OPTIONS]

Read and display ZMQ messages from the flight data topic.

This script subscribes to the ZMQ flight data topic and prints received messages
to the console. It supports both pickled Python objects and binary data formats.

Options:
  -h, --help            Show this help message and exit
  --zmq=PORT            ZMQ port to read from (default: from zmqTopics.topicMavlinkPort)

Configuration:
  ZMQ Port: (from zmqTopics.topicMavlinkPort, or use --zmq to override)
  Topic: (from zmqTopics.topicMavlinkFlightData)
  Host: 127.0.0.1

Examples:
  {script_name}                    # Start reading ZMQ messages on default port
  {script_name} --zmq=7700         # Read from ZMQ port 7700
  {script_name} --help             # Show this help message

Notes:
  - The script will continuously read messages until interrupted (Ctrl+C)
  - Messages are displayed with their type and content
  - Binary messages show hex dump and format detection
  - Make sure the ZMQ publisher (mavlink_to_ZMQ) is running
  - Requires zmqTopics and zmqWrapper modules to be available
""")
    sys.exit(0)

import zmq
import zmqTopics
import zmqWrapper
import time
import pickle
import struct

def print_help():
    """Print help message and exit."""
    script_name = sys.argv[0] if sys.argv else "simpleZMQread.py"
    try:
        port = zmqTopics.topicMavlinkPort
        topic = zmqTopics.topicMavlinkFlightData
        if isinstance(topic, bytes):
            topic_str = topic.decode()
        else:
            topic_str = str(topic)
    except:
        port = "(not available)"
        topic_str = "(not available)"
    
    print(f"""
Usage: {script_name} [OPTIONS]

Read and display ZMQ messages from the flight data topic.

This script subscribes to the ZMQ flight data topic and prints received messages
to the console. It supports both pickled Python objects and binary data formats.

Options:
  -h, --help            Show this help message and exit
  --zmq=PORT            ZMQ port to read from (default: {port})

Configuration:
  Default ZMQ Port: {port}
  Topic: {topic_str}
  Host: 127.0.0.1

Examples:
  {script_name}                    # Start reading ZMQ messages on default port ({port})
  {script_name} --zmq=7700         # Read from ZMQ port 7700
  {script_name} --help             # Show this help message

Notes:
  - The script will continuously read messages until interrupted (Ctrl+C)
  - Messages are displayed with their type and content
  - Binary messages show hex dump and format detection
  - Make sure the ZMQ publisher (mavlink_to_ZMQ) is running
""")
    sys.exit(0)

def main():
    # Help is already checked before imports, but keep this for consistency
    if len(sys.argv) > 1:
        if sys.argv[1] in ['-h', '--help']:
            print_help()
    
    # Parse command line arguments
    zmq_port = None  # Will use default from zmqTopics if not specified
    
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        
        # Check for --zmq option
        if arg.startswith('--zmq='):
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
            print(f"Error: Unknown option: {arg}")
            print("  Use --help for more information")
            sys.exit(1)
        
        i += 1
    
    # Use default port from zmqTopics if not specified
    if zmq_port is None:
        try:
            zmq_port = zmqTopics.topicMavlinkPort
        except:
            print("Error: Could not get default ZMQ port from zmqTopics")
            print("  Please specify port using --zmq=PORT")
            sys.exit(1)
    
    # Validate port range
    if zmq_port < 1 or zmq_port > 65535:
        print(f"Error: Invalid ZMQ port: {zmq_port}")
        print("  Port must be between 1 and 65535")
        sys.exit(1)
    
    print("=" * 60)
    print("Simple ZMQ Reader - Starting...")
    print("=" * 60)
    
    # Create subscription socket for flight data (similar to system_manager.py)
    subs_sock = zmqWrapper.context.socket(zmq.SUB)
    subs_sock.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
    subs_sock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
    subs_sock.connect(f"tcp://127.0.0.1:{zmq_port}")
    subs_sock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicMavlinkFlightData)
    print(f"Subscribed to topic: {zmqTopics.topicMavlinkFlightData} on port: {zmq_port}")
    
    # Give socket time to connect (important for ZMQ PUB/SUB pattern)
    time.sleep(0.2)
    
    print("Waiting for messages... (Press Ctrl+C to stop)")
    print("=" * 60)
    
    message_count = 0
    try:
        while True:
            try:
                # Receive message (single-part with topic prefix)
                msg = subs_sock.recv(zmq.NOBLOCK)
                
                # Strip topic prefix if present
                if msg.startswith(zmqTopics.topicMavlinkFlightData):
                    data_bytes = msg[len(zmqTopics.topicMavlinkFlightData):]
                else:
                    data_bytes = msg
                
                message_count += 1
                
                # Try to deserialize as pickle first (for backward compatibility)
                try:
                    data = pickle.loads(data_bytes)
                    print(f"\n--- Message #{message_count} ---")
                    print(f"Type: {type(data)}")
                    print(f"Data: {data}")
                    if hasattr(data, '__dict__'):
                        print(f"Attributes: {list(data.__dict__.keys())}")
                except (pickle.UnpicklingError, TypeError):
                    # If not pickle, try to parse as binary format
                    print(f"\n--- Message #{message_count} (Binary) ---")
                    print(f"Data length: {len(data_bytes)} bytes")
                    print(f"First 100 bytes (hex): {data_bytes[:100].hex()}")
                    
                    # Try to parse binary format if it looks like flight data
                    if len(data_bytes) >= 8:
                        try:
                            magic, version = struct.unpack('<II', data_bytes[0:8])
                            print(f"Magic: 0x{magic:08X}, Version: {version}")
                            if magic == 0x464C4947:  # "FLIG"
                                print("Detected: Flight Data binary format")
                        except:
                            pass
                
            except zmq.Again:
                # No message available, continue waiting
                time.sleep(0.01)
                continue
            except KeyboardInterrupt:
                print("\n\nStopping...")
                break
            except Exception as e:
                print(f"\nError receiving/processing message: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        subs_sock.close()
        zmqWrapper.context.term()
        print(f"\nTotal messages received: {message_count}")
        print("=" * 60)

if __name__ == '__main__':
    main()
