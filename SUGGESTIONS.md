# ZMQ Architecture Suggestions

## Current Issue: Transient Publisher Problem

When `zmqTakeoffLandAlt.py` runs a one-shot command (takeoff/land), it:
1. Binds to port 7700 as a PUB socket
2. Waits 2 seconds for subscriber connection
3. Sends one message
4. Closes the socket

After this, the `zmq_commands_mavlink` subscriber often enters a broken state where:
- It no longer receives messages from subsequent publishers
- SystemManager commands also stop working
- Only restarting hardware_adapter fixes it

## Root Cause Analysis

1. **ZMQ PUB/SUB Disconnect Handling**: When a publisher disconnects, the subscriber should auto-reconnect when a new publisher binds to the same address. However, ZMQ's internal state may get corrupted under certain timing conditions.

2. **ZMQ_CONFLATE Option** (`zmq_wrapper.c:106`): This option keeps only the latest message. Combined with transient publishers, messages may be lost or the socket may enter an unexpected state.

3. **Socket Close Timing**: The publisher closes immediately after sending. Even with `LINGER=1000`, the subscriber may not have processed the message.

## Suggested Solutions

### Short-term Fixes

1. **Remove ZMQ_CONFLATE** from subscriber (`zmq_wrapper.c:106`):
   ```c
   // Comment out or remove:
   // int conflate = 1;
   // zmq_setsockopt(socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));
   ```

2. **Add Reconnect Logic** in `zmq_commands_mavlink`:
   - Periodically check if messages are being received
   - If no messages for N seconds, reconnect the ZMQ socket

3. **Use Persistent Publisher** instead of one-shot:
   - Modify `zmqTakeoffLandAlt.py` to keep the socket open while waiting for user input
   - Or create a daemon process that acts as command proxy

### Architectural Improvements

1. **Switch PUB/SUB Direction**:
   - Have `zmq_commands_mavlink` BIND (stable server)
   - Have scripts CONNECT (clients)
   - This is the opposite of current design but more reliable for multiple publishers

2. **Use PUSH/PULL Pattern** for commands:
   - Better for one-shot command delivery
   - Guarantees message delivery (no subscription issues)

3. **Add XPUB/XSUB Proxy**:
   - Run a persistent proxy that bridges publishers and subscribers
   - Publishers connect to proxy's XSUB socket
   - Subscriber connects to proxy's XPUB socket
   - Proxy handles all reconnection logic

### Implementation Priority

1. **Quick Win**: Remove `ZMQ_CONFLATE` and increase delays
2. **Medium Effort**: Add reconnect logic to subscriber
3. **Long-term**: Consider PUSH/PULL or XPUB/XSUB architecture

## Current Workaround

Until a proper fix is implemented:
- Restart `hardware_adapter` after using `zmqTakeoffLandAlt.py`
- Or use `mavlinkTakeoffLandAlt.py` which bypasses ZMQ entirely
