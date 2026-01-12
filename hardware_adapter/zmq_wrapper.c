#define _DEFAULT_SOURCE
#include "zmq_wrapper.h"
#include "zmq_topics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

static void* zmq_context = NULL;

void zmq_wrapper_init(void) {
    if (zmq_context == NULL) {
        zmq_context = zmq_ctx_new();
        if (zmq_context == NULL) {
            fprintf(stderr, "Failed to create ZMQ context\n");
            exit(1);
        }
    }
}

void zmq_wrapper_cleanup(void) {
    if (zmq_context != NULL) {
        zmq_ctx_destroy(zmq_context);
        zmq_context = NULL;
    }
}

void* zmq_wrapper_get_context(void) {
    return zmq_context;
}

// Publisher functions
void* zmq_publisher_create(int port) {
    void* context = zmq_wrapper_get_context();
    if (context == NULL) {
        zmq_wrapper_init();
        context = zmq_wrapper_get_context();
    }
    
    void* socket = zmq_socket(context, ZMQ_PUB);
    if (socket == NULL) {
        fprintf(stderr, "Failed to create publisher socket: %s\n", zmq_strerror(zmq_errno()));
        return NULL;
    }
    
    char address[64];
    snprintf(address, sizeof(address), "tcp://*:%d", port);
    
    if (zmq_bind(socket, address) != 0) {
        fprintf(stderr, "Failed to bind publisher to %s: %s\n", address, zmq_strerror(zmq_errno()));
        zmq_close(socket);
        return NULL;
    }
    
    // Give ZMQ time to bind
    usleep(100000);  // 100ms
    
    return socket;
}

void zmq_publisher_destroy(void* socket) {
    if (socket != NULL) {
        zmq_close(socket);
    }
}

int zmq_publisher_send(void* socket, const char* topic, const void* data, size_t data_len) {
    if (socket == NULL || topic == NULL || data == NULL) {
        return -1;
    }
    
    // Send topic + data as single message (for CONFLATE support)
    size_t topic_len = strlen(topic);
    size_t total_len = topic_len + data_len;
    char* buffer = (char*)malloc(total_len);
    if (buffer == NULL) {
        return -1;
    }
    
    memcpy(buffer, topic, topic_len);
    memcpy(buffer + topic_len, data, data_len);
    
    int result = zmq_send(socket, buffer, total_len, ZMQ_DONTWAIT);
    free(buffer);
    
    return result;
}

// Subscriber functions
void* zmq_subscriber_create(int port) {
    void* context = zmq_wrapper_get_context();
    if (context == NULL) {
        zmq_wrapper_init();
        context = zmq_wrapper_get_context();
    }
    
    void* socket = zmq_socket(context, ZMQ_SUB);
    if (socket == NULL) {
        fprintf(stderr, "Failed to create subscriber socket: %s\n", zmq_strerror(zmq_errno()));
        return NULL;
    }
    
    // Set CONFLATE option to keep only latest message
    int conflate = 1;
    zmq_setsockopt(socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    
    // Set receive timeout (but ZMQ_DONTWAIT will override this in receive calls)
    int timeout = 100;  // 100ms
    zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    
    char address[64];
    snprintf(address, sizeof(address), "tcp://127.0.0.1:%d", port);
    
    if (zmq_connect(socket, address) != 0) {
        fprintf(stderr, "Failed to connect subscriber to %s: %s\n", address, zmq_strerror(zmq_errno()));
        zmq_close(socket);
        return NULL;
    }
    
    printf("Hardware_adapter: ZMQ subscriber connected to %s\n", address);
    
    return socket;
}

void zmq_subscriber_destroy(void* socket) {
    if (socket != NULL) {
        zmq_close(socket);
    }
}

int zmq_subscriber_subscribe(void* socket, const char* topic) {
    if (socket == NULL) {
        return -1;
    }
    
    // Empty string means subscribe to all messages
    size_t topic_len = (topic == NULL) ? 0 : strlen(topic);
    int result = zmq_setsockopt(socket, ZMQ_SUBSCRIBE, topic == NULL ? "" : topic, topic_len);
    if (result == 0) {
        if (topic == NULL || topic_len == 0) {
            printf("Hardware_adapter: Successfully subscribed to ALL messages (empty filter)\n");
        } else {
            printf("Hardware_adapter: Successfully subscribed to topic: %s (length: %zu)\n", topic, topic_len);
        }
    } else {
        fprintf(stderr, "Hardware_adapter: Failed to subscribe to topic %s: %s\n", 
                topic == NULL ? "(empty)" : topic, zmq_strerror(zmq_errno()));
    }
    return result;
}

int zmq_subscriber_receive(void* socket, char* topic_buffer, size_t topic_buffer_size,
                          void* data_buffer, size_t data_buffer_size, int timeout_ms) {
    if (socket == NULL || topic_buffer == NULL || data_buffer == NULL) {
        return -1;
    }
    
    // Set timeout
    int timeout = timeout_ms;
    zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    
    // Receive multipart message: first frame is topic, second frame is data
    zmq_msg_t topic_msg;
    zmq_msg_init(&topic_msg);
    
    // Use blocking receive if timeout > 0, otherwise non-blocking
    int flags = (timeout_ms > 0) ? 0 : ZMQ_DONTWAIT;
    int result = zmq_msg_recv(&topic_msg, socket, flags);
    if (result < 0) {
        int err = zmq_errno();
        // Only log errors occasionally to avoid spam (ETIMEDOUT is expected when no messages)
        static int error_count = 0;
        static int eagain_count = 0;
        static int etimedout_count = 0;
        if (err == EAGAIN) {
            eagain_count++;
            // Log EAGAIN occasionally to verify we're actually trying to receive
            if (eagain_count % 10000 == 0) {
                // This is normal - just means no message available
            }
        } else if (err == ETIMEDOUT) {
            etimedout_count++;
            // This is also normal when timeout is set
        } else {
            if (error_count++ % 1000 == 0) {
                fprintf(stderr, "Hardware_adapter: zmq_msg_recv error: %s (errno=%d)\n", zmq_strerror(err), err);
            }
        }
        zmq_msg_close(&topic_msg);
        return -1;
    }
    
    // Check if there's more data (multipart message)
    // Note: ZMQ_RCVMORE must be checked on the MESSAGE, not the socket
    int more = 0;
    size_t more_size = sizeof(more);
    zmq_getsockopt(socket, ZMQ_RCVMORE, &more, &more_size);
    
    // Also check the message itself for multipart flag
    int msg_more = zmq_msg_more(&topic_msg);
    
    if (!more && !msg_more) {
        // Single-part message - could be legacy format or data-only frame
        size_t msg_size = zmq_msg_size(&topic_msg);
        const char* msg_data = (const char*)zmq_msg_data(&topic_msg);
        
        // Check if this looks like binary serialized data (starts with magic number)
        // Magic numbers: VELC=0x56454C43 (little-endian: 0x43 0x4C 0x45 0x56), ATTC=0x41545443
        // Read magic number as little-endian (matches Python struct.pack('<I', ...))
        uint32_t magic = 0;
        if (msg_size >= 4) {
            // Read as little-endian (Python uses '<I' format)
            magic = ((uint32_t)(unsigned char)msg_data[0]) |
                   ((uint32_t)(unsigned char)msg_data[1] << 8) |
                   ((uint32_t)(unsigned char)msg_data[2] << 16) |
                   ((uint32_t)(unsigned char)msg_data[3] << 24);
        }
        
        // Check for pickle format (starts with PROTO opcode 0x80 followed by version 0x04 or 0x05)
        // Pickle format: 0x80 (PROTO), 0x04/0x05 (version), ...
        // Note: Cast to unsigned char to handle signed char systems
        if (msg_size >= 2 && (unsigned char)msg_data[0] == 0x80 && 
            ((unsigned char)msg_data[1] == 0x04 || (unsigned char)msg_data[1] == 0x05)) {
            // This is pickled data - need to determine which command type by searching for keys
            // Search for command-specific keys in the pickle data
            const char* takeoff_key = "takeoff_altitude";
            const char* acc_key = "accCmd";
            const char* arm_key = "arm";  // ARM commands might also be pickled
            
            const char* assumed_topic = NULL;
            
            // Check for takeoff command key
            for (size_t i = 0; i < msg_size - 15; i++) {
                if (memcmp(msg_data + i, takeoff_key, 15) == 0) {
                    assumed_topic = TOPIC_GUIDANCE_CMD_TAKEOFF;
                    break;
                }
            }
            
            // If not takeoff, check for ACC command key
            if (assumed_topic == NULL) {
                for (size_t i = 0; i < msg_size - 6; i++) {
                    if (memcmp(msg_data + i, acc_key, 6) == 0) {
                        assumed_topic = TOPIC_GUIDANCE_CMD_ACC;
                        break;
                    }
                }
            }
            
            // Default to ACC if we can't determine (backward compatibility)
            if (assumed_topic == NULL) {
                assumed_topic = TOPIC_GUIDANCE_CMD_ACC;
            }
            
            
            // Use detected topic
            size_t topic_len = strlen(assumed_topic);
            size_t copy_len = (topic_len < topic_buffer_size - 1) ? topic_len : topic_buffer_size - 1;
            memcpy(topic_buffer, assumed_topic, copy_len);
            topic_buffer[copy_len] = '\0';
            
            // Copy data
            size_t data_len = msg_size;
            if (data_len > data_buffer_size) {
                data_len = data_buffer_size;
            }
            memcpy(data_buffer, msg_data, data_len);
            
            zmq_msg_close(&topic_msg);
            return (int)data_len;
        }
        
        // Check for VELC (0x56454C43) or ATTC (0x41545443) magic numbers
        if (magic == 0x56454C43 || magic == 0x41545443) {
            // This is binary serialized data without topic prefix
            // This means ZMQ filtered out the topic frame, or we're receiving only the data frame
            // Try to determine topic from context or assume it's the most common one
            static int data_only_count = 0;
            data_only_count++;
            
            // For now, assume velocity command if magic is VELC
            const char* assumed_topic = (magic == 0x56454C43) ? TOPIC_GUIDANCE_CMD_VEL_NED : TOPIC_GUIDANCE_CMD_ATTITUDE;
            
            if (data_only_count <= 3) {
                printf("Hardware_adapter: WARNING - Received data-only frame (no topic). Magic=0x%08X, assuming topic=%s\n", 
                       magic, assumed_topic);
                printf("  This suggests ZMQ subscription filter may be stripping topic frame.\n");
                printf("  Consider subscribing to empty string to receive full multipart messages.\n");
            }
            
            // Copy assumed topic
            size_t topic_len = strlen(assumed_topic);
            size_t copy_len = (topic_len < topic_buffer_size - 1) ? topic_len : topic_buffer_size - 1;
            memcpy(topic_buffer, assumed_topic, copy_len);
            topic_buffer[copy_len] = '\0';
            
            // Copy data
            size_t data_len = msg_size;
            if (data_len > data_buffer_size) {
                data_len = data_buffer_size;
            }
            memcpy(data_buffer, msg_data, data_len);
            
            zmq_msg_close(&topic_msg);
            return (int)data_len;
        }
        
        // Try to find topic prefix in single-part message (legacy format)
        const char* topics[] = {
            TOPIC_GUIDANCE_CMD_ATTITUDE,
            TOPIC_GUIDANCE_CMD_VEL_NED,
            TOPIC_GUIDANCE_CMD_VEL_BODY,
            TOPIC_GUIDANCE_CMD_ACC,
            TOPIC_GUIDANCE_CMD_ARM,
            TOPIC_GUIDANCE_CMD_TAKEOFF,
            TOPIC_GUIDANCE_CMD_LAND
        };
        
        const char* found_topic = NULL;
        size_t topic_len = 0;
        
        for (int i = 0; i < 7; i++) {
            size_t len = strlen(topics[i]);
            if (msg_size >= len && memcmp(msg_data, topics[i], len) == 0) {
                found_topic = topics[i];
                topic_len = len;
                break;
            }
        }
        
        if (found_topic == NULL) {
            // Check if this is an 8-byte message (could be takeoff altitude as double)
            // This happens when ZMQ filters out the topic frame for takeoff/land commands
            if (msg_size == sizeof(double)) {
                // This is likely a takeoff or land command with just the altitude/parameter
                // Since we can't distinguish without the topic, we'll try to receive a second frame
                // in case it's actually a multipart message
                zmq_msg_t data_msg;
                zmq_msg_init(&data_msg);
                int second_result = zmq_msg_recv(&data_msg, socket, ZMQ_DONTWAIT);
                
                if (second_result >= 0) {
                    // We got a second frame - the first was actually the topic!
                    // This means the topic frame was received but RCVMORE was wrong
                    size_t topic_size = zmq_msg_size(&topic_msg);
                    size_t data_size = zmq_msg_size(&data_msg);
                    
                    // First frame should be topic, second should be data
                    const char* topic_candidate = (const char*)zmq_msg_data(&topic_msg);
                    const char* data_candidate = (const char*)zmq_msg_data(&data_msg);
                    
                    // Check if first frame looks like a topic string
                    bool looks_like_topic = false;
                    for (size_t i = 0; i < topic_size && i < 50; i++) {
                        if (topic_candidate[i] >= 32 && topic_candidate[i] < 127) {
                            looks_like_topic = true;
                        } else if (topic_candidate[i] != 0) {
                            looks_like_topic = false;
                            break;
                        }
                    }
                    
                    if (looks_like_topic && data_size == sizeof(double)) {
                        // First frame is topic, second is data (altitude)
                        // Verify topic matches expected topics
                        const char* topics_check[] = {
                            TOPIC_GUIDANCE_CMD_TAKEOFF,
                            TOPIC_GUIDANCE_CMD_LAND
                        };
                        
                        for (int i = 0; i < 2; i++) {
                            size_t len = strlen(topics_check[i]);
                            if (topic_size == len && memcmp(topic_candidate, topics_check[i], len) == 0) {
                                // Found matching topic
                                size_t copy_len = (len < topic_buffer_size - 1) ? len : topic_buffer_size - 1;
                                memcpy(topic_buffer, topics_check[i], copy_len);
                                topic_buffer[copy_len] = '\0';
                                
                                size_t data_copy_len = (data_size < data_buffer_size) ? data_size : data_buffer_size;
                                memcpy(data_buffer, data_candidate, data_copy_len);
                                
                                zmq_msg_close(&topic_msg);
                                zmq_msg_close(&data_msg);
                                return (int)data_copy_len;
                            }
                        }
                    }
                    
                    // If we got here, the frames might be swapped or something else
                    zmq_msg_close(&data_msg);
                }
                
                // If no second frame or it didn't match, assume it's takeoff altitude
                // (8 bytes = double, which matches takeoff/land parameter format)
            // Assume takeoff command (most common use case)
            size_t topic_len = strlen(TOPIC_GUIDANCE_CMD_TAKEOFF);
            size_t copy_len = (topic_len < topic_buffer_size - 1) ? topic_len : topic_buffer_size - 1;
            memcpy(topic_buffer, TOPIC_GUIDANCE_CMD_TAKEOFF, copy_len);
            topic_buffer[copy_len] = '\0';
            
            // Copy the 8-byte double data
            size_t data_copy_len = (msg_size < data_buffer_size) ? msg_size : data_buffer_size;
            memcpy(data_buffer, msg_data, data_copy_len);
                
                zmq_msg_close(&topic_msg);
                return (int)data_copy_len;
            }
            
            // Debug: Log ALL unmatched messages
            static int unmatched_count = 0;
            unmatched_count++;
            size_t debug_len = (msg_size < 50) ? msg_size : 50;
            printf("Hardware_adapter: Received single-part message #%d with unknown topic prefix (size=%zu, first %zu bytes: ", unmatched_count, msg_size, debug_len);
            for (size_t i = 0; i < debug_len; i++) {
                if (msg_data[i] >= 32 && msg_data[i] < 127) {
                    printf("%c", msg_data[i]);
                } else {
                    printf("\\x%02x", (unsigned char)msg_data[i]);
                }
            }
            printf(")\n");
            zmq_msg_close(&topic_msg);
            return -2;
        }
        
        // Copy topic
        size_t copy_len = (topic_len < topic_buffer_size - 1) ? topic_len : topic_buffer_size - 1;
        memcpy(topic_buffer, found_topic, copy_len);
        topic_buffer[copy_len] = '\0';
        
        // Copy data
        size_t data_len = msg_size - topic_len;
        if (data_len > data_buffer_size) {
            data_len = data_buffer_size;
        }
        memcpy(data_buffer, msg_data + topic_len, data_len);
        
        zmq_msg_close(&topic_msg);
        return (int)data_len;
    }
    
    // Multipart message: topic is first frame, data is second frame
    size_t topic_size = zmq_msg_size(&topic_msg);
    const char* topic_data = (const char*)zmq_msg_data(&topic_msg);
    
    // Verify topic matches expected topics
    const char* topics[] = {
        TOPIC_GUIDANCE_CMD_ATTITUDE,
        TOPIC_GUIDANCE_CMD_VEL_NED,
        TOPIC_GUIDANCE_CMD_VEL_BODY,
        TOPIC_GUIDANCE_CMD_ACC,
        TOPIC_GUIDANCE_CMD_ARM,
        TOPIC_GUIDANCE_CMD_TAKEOFF,
        TOPIC_GUIDANCE_CMD_LAND
    };
    
    const char* found_topic = NULL;
    for (int i = 0; i < 7; i++) {
        size_t len = strlen(topics[i]);
        if (topic_size == len && memcmp(topic_data, topics[i], len) == 0) {
            found_topic = topics[i];
            break;
        }
    }
    
    if (found_topic == NULL) {
        // Debug: Log unmatched topic
        static int unmatched_topic_count = 0;
        unmatched_topic_count++;
        size_t debug_len = (topic_size < 50) ? topic_size : 50;
        printf("Hardware_adapter: Received multipart message #%d with unknown topic (size=%zu, first %zu bytes: ", unmatched_topic_count, topic_size, debug_len);
        for (size_t i = 0; i < debug_len; i++) {
            if (topic_data[i] >= 32 && topic_data[i] < 127) {
                printf("%c", topic_data[i]);
            } else {
                printf("\\x%02x", (unsigned char)topic_data[i]);
            }
        }
        printf(")\n");
        zmq_msg_close(&topic_msg);
        return -2;
    }
    
    // Copy topic (null-terminated)
    size_t copy_len = (topic_size < topic_buffer_size - 1) ? topic_size : topic_buffer_size - 1;
    memcpy(topic_buffer, found_topic, copy_len);
    topic_buffer[copy_len] = '\0';
    
    // Receive data frame
    zmq_msg_t data_msg;
    zmq_msg_init(&data_msg);
    result = zmq_msg_recv(&data_msg, socket, 0);  // Blocking receive for second frame
    if (result < 0) {
        int err = zmq_errno();
        static int data_frame_error_count = 0;
        if (data_frame_error_count++ % 100 == 0) {
            fprintf(stderr, "Hardware_adapter: Failed to receive data frame: %s (errno=%d)\n", zmq_strerror(err), err);
        }
        zmq_msg_close(&topic_msg);
        zmq_msg_close(&data_msg);
        return -1;
    }
    
    // Copy data
    size_t data_len = zmq_msg_size(&data_msg);
    if (data_len > data_buffer_size) {
        data_len = data_buffer_size;
    }
    memcpy(data_buffer, zmq_msg_data(&data_msg), data_len);
    
    zmq_msg_close(&topic_msg);
    zmq_msg_close(&data_msg);
    return (int)data_len;
}

