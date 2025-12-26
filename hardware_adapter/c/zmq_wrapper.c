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
    
    zmq_msg_t msg;
    zmq_msg_init(&msg);
    
    // Use blocking receive if timeout > 0, otherwise non-blocking
    int flags = (timeout_ms > 0) ? 0 : ZMQ_DONTWAIT;
    int result = zmq_msg_recv(&msg, socket, flags);
    if (result < 0) {
        int err = zmq_errno();
        // Only log errors occasionally to avoid spam (ETIMEDOUT is expected when no messages)
        static int error_count = 0;
        if (err != EAGAIN && err != ETIMEDOUT) {
            if (error_count++ % 1000 == 0) {
                fprintf(stderr, "Hardware_adapter: zmq_msg_recv error: %s (errno=%d)\n", zmq_strerror(err), err);
            }
        }
        zmq_msg_close(&msg);
        return -1;
    }
    
    size_t msg_size = zmq_msg_size(&msg);
    const char* msg_data = (const char*)zmq_msg_data(&msg);
    
    // Find topic prefix
    const char* topics[] = {
        TOPIC_GUIDANCE_CMD_ATTITUDE,
        TOPIC_GUIDANCE_CMD_VEL_NED,
        TOPIC_GUIDANCE_CMD_VEL_BODY,
        TOPIC_GUIDANCE_CMD_ACC,
        TOPIC_GUIDANCE_CMD_ARM
    };
    
    const char* found_topic = NULL;
    size_t topic_len = 0;
    
    for (int i = 0; i < 5; i++) {
        size_t len = strlen(topics[i]);
        if (msg_size >= len && memcmp(msg_data, topics[i], len) == 0) {
            found_topic = topics[i];
            topic_len = len;
            break;
        }
    }
    
    if (found_topic == NULL) {
        // Debug: Log ALL unmatched messages (this is important for debugging)
        static int unmatched_count = 0;
        unmatched_count++;
        // Print first 50 bytes of message for debugging
        size_t debug_len = (msg_size < 50) ? msg_size : 50;
        printf("Hardware_adapter: Received message #%d with unknown topic prefix (size=%zu, first %zu bytes: ", unmatched_count, msg_size, debug_len);
        for (size_t i = 0; i < debug_len; i++) {
            if (msg_data[i] >= 32 && msg_data[i] < 127) {
                printf("%c", msg_data[i]);
            } else {
                printf("\\x%02x", (unsigned char)msg_data[i]);
            }
        }
        printf(")\n");
        printf("Hardware_adapter: Expected topics: %s, %s, %s, %s, %s\n",
                TOPIC_GUIDANCE_CMD_ATTITUDE, TOPIC_GUIDANCE_CMD_VEL_NED,
                TOPIC_GUIDANCE_CMD_VEL_BODY, TOPIC_GUIDANCE_CMD_ACC, TOPIC_GUIDANCE_CMD_ARM);
        zmq_msg_close(&msg);
        // Return a special value to indicate message was received but unmatched
        // We'll use -2 to distinguish from -1 (no message)
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
    
    zmq_msg_close(&msg);
    return (int)data_len;
}

