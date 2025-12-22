#!/usr/bin/env python3
"""
Simple diagnostic script to test ZMQ communication between hardware_adapter and system_manager.
Run this to verify both processes can communicate.
"""
import zmq
import zmqTopics
import time
import pickle

def test_flight_data_reception():
    """Test if we can receive flight data from hardware_adapter"""
    print("=" * 60)
    print("Testing Flight Data Reception (hardware_adapter -> system_manager)")
    print("=" * 60)
    
    context = zmq.Context()
    sub_sock = context.socket(zmq.SUB)
    sub_sock.setsockopt(zmq.CONFLATE, 1)  # Enable CONFLATE for single-part messages
    sub_sock.connect(f"tcp://127.0.0.1:{zmqTopics.topicMavlinkPort}")
    sub_sock.setsockopt(zmq.SUBSCRIBE, zmqTopics.topicMavlinkFlightData)
    sub_sock.setsockopt(zmq.RCVTIMEO, 2000)  # 2 second timeout
    
    print(f"Subscribed to flight data on port {zmqTopics.topicMavlinkPort}")
    print("Waiting 3 seconds for messages from hardware_adapter...")
    print("  (Note: In ZMQ PUB/SUB, subscriber must connect AFTER publisher binds)")
    print("  (Using single-part messages with CONFLATE enabled)")
    
    # Give time for any existing publisher to be ready
    time.sleep(0.5)
    
    received_count = 0
    start_time = time.monotonic()
    last_msg_time = start_time
    while time.monotonic() - start_time < 3.0:
        try:
            msg = sub_sock.recv(zmq.NOBLOCK)  # Single-part message with topic prefix
            received_count += 1
            last_msg_time = time.monotonic()
            if received_count == 1:
                print(f"✓ SUCCESS: Received first message from hardware_adapter!")
                print(f"  Message length: {len(msg)} bytes (single-part with topic prefix)")
                if msg.startswith(zmqTopics.topicMavlinkFlightData):
                    data_len = len(msg) - len(zmqTopics.topicMavlinkFlightData)
                    print(f"  Topic: {zmqTopics.topicMavlinkFlightData}, Data: {data_len} bytes")
        except zmq.Again:
            time.sleep(0.01)
        except Exception as e:
            print(f"✗ ERROR receiving message: {e}")
            import traceback
            traceback.print_exc()
            break
    
    # Check if we received messages recently
    if received_count > 0:
        time_since_last = time.monotonic() - last_msg_time
        print(f"  Last message received {time_since_last:.2f}s ago")
        print(f"✓ SUCCESS: Received {received_count} messages from hardware_adapter")
        print(f"  Hardware_adapter is publishing correctly!")
        print(f"  Rate: ~{received_count/3:.0f} messages/second")
    else:
        print(f"✗ FAILED: No messages received from hardware_adapter")
        print(f"  Make sure hardware_adapter is running and publishing on port {zmqTopics.topicMavlinkPort}")
        print(f"  Note: In ZMQ PUB/SUB, subscriber must connect AFTER publisher binds")
        print(f"  If hardware_adapter started after this test, restart hardware_adapter and run test again")
    
    sub_sock.close()
    context.term()
    return received_count > 0

def test_command_publishing():
    """Test if we can publish commands (system_manager -> hardware_adapter)"""
    print("\n" + "=" * 60)
    print("Testing Command Publishing (system_manager -> hardware_adapter)")
    print("=" * 60)
    
    context = zmq.Context()
    pub_sock = context.socket(zmq.PUB)
    pub_sock.bind(f"tcp://127.0.0.1:{zmqTopics.topicGuidenceCmdPort}")
    
    print(f"Publisher bound to port {zmqTopics.topicGuidenceCmdPort}")
    print("Waiting 1 second for subscribers to connect...")
    time.sleep(1.0)
    
    # Send a test message
    test_msg = {
        'ts': time.monotonic(),
        'velCmd': [0.1, 0.0, 0.0],
        'yawCmd': 0.0,
        'yawRateCmd': 0.0
    }
    
    try:
        # Send as single-part message with topic prefix
        pub_sock.send(zmqTopics.topicGuidenceCmdVelNed + pickle.dumps(test_msg))
        print(f"✓ SUCCESS: Sent test command message")
        print(f"  If hardware_adapter is running, it should receive this message")
        print(f"  (Using single-part message with topic prefix)")
    except Exception as e:
        print(f"✗ ERROR: Failed to send message: {e}")
    
    pub_sock.close()
    context.term()
    return True

if __name__ == "__main__":
    print("\nZMQ Connection Diagnostic Tool")
    print("This script tests communication between hardware_adapter and system_manager\n")
    
    # Test flight data reception
    flight_data_ok = test_flight_data_reception()
    
    # Test command publishing
    command_pub_ok = test_command_publishing()
    
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"Flight Data Reception (hardware_adapter -> system_manager): {'✓ OK' if flight_data_ok else '✗ FAILED'}")
    print(f"Command Publishing (system_manager -> hardware_adapter): {'✓ OK' if command_pub_ok else '✗ FAILED'}")
    
    if not flight_data_ok:
        print("\n⚠️  Make sure hardware_adapter is running:")
        print("   cd /home/valentin/RL/src && python hardware_adapter.py")
    
    if not command_pub_ok:
        print("\n⚠️  Make sure system_manager is running:")
        print("   cd /home/valentin/RL/src && python system_manager.py")
    
    print("\n" + "=" * 60)

