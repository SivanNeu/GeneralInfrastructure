#!/usr/bin/env python3
"""
Simple ZMQ reader program that reads data from ZMQ port and prints it to screen.
Based on system_manager.py reference implementation.
"""
import zmq
import zmqTopics
import zmqWrapper
import time
import pickle
import struct

def main():
    print("=" * 60)
    print("Simple ZMQ Reader - Starting...")
    print("=" * 60)
    
    # Create subscription socket for flight data (similar to system_manager.py)
    subs_sock = zmqWrapper.context.socket(zmq.SUB)
    subs_sock.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
    subs_sock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
    subs_sock.connect(f"tcp://127.0.0.1:{zmqTopics.topicMavlinkPort}")
    subs_sock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicMavlinkFlightData)
    print(f"Subscribed to topic: {zmqTopics.topicMavlinkFlightData} on port: {zmqTopics.topicMavlinkPort}")
    
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
